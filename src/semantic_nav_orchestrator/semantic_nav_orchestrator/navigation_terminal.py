"""Unified interactive terminal for semantic navigation.

Replaces both the one-shot orchestrator CLI and operator_io_node.
Do NOT run operator_io_node alongside this terminal.

Thread model:
  input_thread  — blocks on input(); puts commands on _cmd_q
  main thread   — controller loop; picks commands from _cmd_q;
                  calls navigate() which polls for done / operator
                  prompts / preemption all in one place
  ros_thread    — executor.spin(); handles service callbacks and subs

Preemption:
  The user may type a new command at any time. The input_thread
  puts it on _cmd_q. navigate() drains _cmd_q, fires /cancel_navigation,
  waits for the current future to resolve, then returns the new command
  so the controller loop can start a fresh goal without going back to
  blocking input().
"""

from __future__ import annotations

import queue
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Trigger

from semantic_nav_interfaces.srv import (
    NavigateToQuery,
    OperatorDecision,
    ParseSemanticCommand,
)
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# ANSI colour helpers (suppressed when stdout is not a TTY)
# ---------------------------------------------------------------------------
_TTY = sys.stdout.isatty()

def _c(code: str, text: str) -> str:
    return f"\033[{code}m{text}\033[0m" if _TTY else text

def bold(t: str) -> str:     return _c("1", t)
def cyan(t: str) -> str:     return _c("36", t)
def green(t: str) -> str:    return _c("32", t)
def red(t: str) -> str:      return _c("31", t)
def yellow(t: str) -> str:   return _c("33", t)
def dim(t: str) -> str:      return _c("2", t)

_OBJECT_KEY_RE = re.compile(r"[a-z][a-z0-9 _]*:\d+", re.IGNORECASE)


def _looks_like_object_key(s: str) -> bool:
    return bool(_OBJECT_KEY_RE.fullmatch(s.strip().lower()))


# ---------------------------------------------------------------------------
# Result carrier
# ---------------------------------------------------------------------------
@dataclass
class NavResult:
    success: bool
    outcome: str
    failure_reason: str
    preempt_cmd: Optional[str] = None   # non-None → user typed new command
    exit_requested: bool = False         # user typed Ctrl-D / sent None


# ---------------------------------------------------------------------------
# Terminal node
# ---------------------------------------------------------------------------
class NavigationTerminal(Node):

    def __init__(self) -> None:
        super().__init__("navigation_terminal")

        cbg = ReentrantCallbackGroup()

        # Operator decision: BT calls /operator_decision (on ROS thread).
        # We funnel it to main thread via queues so stdin is used from one
        # place only.
        self._op_req_q: queue.Queue = queue.Queue()
        self._op_resp_q: queue.Queue = queue.Queue()

        self._operator_srv = self.create_service(
            OperatorDecision,
            "/operator_decision",
            self._cb_operator_decision,
            callback_group=cbg,
        )

        # Clients
        self._nav_client = self.create_client(
            NavigateToQuery, "/navigate_to_query", callback_group=cbg
        )
        self._cancel_client = self.create_client(
            Trigger, "/cancel_navigation", callback_group=cbg
        )
        self._parse_client = self.create_client(
            ParseSemanticCommand, "/parse_semantic_command", callback_group=cbg
        )

        # Recovery status subscription — prints live updates during navigation
        self._status_sub = self.create_subscription(
            String, "/recovery_status", self._cb_status, 10
        )

        self._nav_active = False

    # ------------------------------------------------------------------
    # ROS callbacks (called from ROS spin thread)
    # ------------------------------------------------------------------

    def _cb_operator_decision(
        self,
        request: OperatorDecision.Request,
        response: OperatorDecision.Response,
    ) -> OperatorDecision.Response:
        """Block the ROS callback until main thread handles operator input."""
        self._op_req_q.put(request)
        ack, note = self._op_resp_q.get()
        response.acknowledged = ack
        response.operator_note = note
        return response

    # Map raw FSM/BT status strings → simplified labels shown to the user.
    # Strings with a "|reason=…" suffix are matched by prefix.
    _STATUS_LABELS: dict = {
        "EXECUTING": "EXECUTING",
        "BT_RECOVERY_DIRECTIVE_IN_PROGRESS": "RECOVERY_EXECUTING",
    }
    # Statuses that carry no user-visible meaning (internal plumbing).
    _STATUS_SILENT = frozenset({
        "RECOVERY_IDLE",
        "BT_RECOVERY_DIRECTIVE_READY",
        "TERMINAL_SUCCESS",
    })

    def _cb_status(self, msg: String) -> None:
        if not self._nav_active:
            return
        raw = msg.data.split("|")[0]  # strip "|reason=…" suffix
        if raw in self._STATUS_SILENT:
            return
        label = self._STATUS_LABELS.get(raw, raw)
        _emit(dim(f"  · {label}"))

    # ------------------------------------------------------------------
    # Navigation (called from main thread)
    # ------------------------------------------------------------------

    def navigate(
        self,
        query: str,
        nl_command: str,
        cmd_q: queue.Queue,
    ) -> NavResult:
        """
        Send a /navigate_to_query goal and block until:
          - navigation completes   → NavResult(success=..., preempt_cmd=None)
          - user types new command → NavResult(outcome=PREEMPTED, preempt_cmd=new)
          - user signals exit      → NavResult(exit_requested=True)

        If the orchestrator is momentarily locked (BUSY after a cancel) we
        retry a few times with a short back-off before giving up.
        """
        req = NavigateToQuery.Request()
        req.query = query
        req.nl_command = nl_command

        future = None
        for attempt in range(12):
            if not self._nav_client.wait_for_service(timeout_sec=3.0):
                return NavResult(
                    success=False,
                    outcome="SERVICE_UNAVAILABLE",
                    failure_reason="/navigate_to_query not available",
                )
            future = self._nav_client.call_async(req)
            # Wait briefly for a BUSY response before retrying
            deadline = time.monotonic() + 0.3
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if future.done() and future.result().outcome == "BUSY":
                time.sleep(0.15)
                future = None
                continue
            break

        if future is None:
            return NavResult(
                success=False,
                outcome="BUSY",
                failure_reason="Orchestrator did not accept goal after cancel",
            )

        self._nav_active = True

        try:
            while not future.done():
                # --- operator decision prompt ---
                try:
                    op_req = self._op_req_q.get_nowait()
                    self._handle_operator_prompt(op_req)
                    continue
                except queue.Empty:
                    pass

                # --- preemption / exit ---
                try:
                    new_cmd = cmd_q.get_nowait()
                    if new_cmd is None:
                        self._fire_cancel()
                        self._drain_future(future)
                        return NavResult(
                            success=False,
                            outcome="CANCELLED",
                            failure_reason="User exit",
                            exit_requested=True,
                        )
                    _emit(yellow(f"\n⚡ Preempted → new command: {bold(repr(new_cmd))}"))
                    self._fire_cancel()
                    self._drain_future(future)
                    return NavResult(
                        success=False,
                        outcome="PREEMPTED",
                        failure_reason="",
                        preempt_cmd=new_cmd,
                    )
                except queue.Empty:
                    pass

                time.sleep(0.05)
        finally:
            self._nav_active = False

        resp = future.result()
        return NavResult(
            success=resp.success,
            outcome=resp.outcome,
            failure_reason=resp.failure_reason,
        )

    def _fire_cancel(self) -> None:
        if self._cancel_client.service_is_ready():
            self._cancel_client.call_async(Trigger.Request())  # fire-and-forget

    def _drain_future(self, future, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

    # ------------------------------------------------------------------
    # NL parsing (called from main thread)
    # ------------------------------------------------------------------

    def parse_nl(self, command: str) -> Optional[str]:
        """
        Call /parse_semantic_command.
        Returns the resolved query string (object_key or tag), or None on
        failure (already prints the reason).
        """
        if not self._parse_client.wait_for_service(timeout_sec=5.0):
            _emit(red("  [!] LLM parse service unavailable — is navigator_node running?"))
            return None

        req = ParseSemanticCommand.Request()
        req.command = command
        future = self._parse_client.call_async(req)

        deadline = time.monotonic() + 60.0
        while not future.done():
            if time.monotonic() > deadline:
                _emit(red("  [!] LLM parse timed out"))
                return None
            time.sleep(0.05)

        resp = future.result()
        if not resp.success or resp.intent != "navigate_to_object":
            _emit(red(f"  [!] LLM rejected: intent={resp.intent}  {resp.message}"))
            return None

        query = resp.target_object_key if resp.target_object_key else resp.object_tag
        conf = resp.confidence_percent
        _emit(cyan(f"  → Parsed: {bold(query)}  confidence={conf}%"))
        return query

    # ------------------------------------------------------------------
    # Operator prompt (called from main thread while in navigate() loop)
    # ------------------------------------------------------------------

    def _handle_operator_prompt(self, request: OperatorDecision.Request) -> None:
        _emit("")
        _emit(yellow("  ╔══════════════════════════════════════════════════╗"))
        _emit(yellow(f"  ║ OPERATOR PROMPT                                  ║"))
        _emit(yellow("  ╚══════════════════════════════════════════════════╝"))
        _emit(yellow(f"  {request.prompt_text}"))
        _emit(yellow(f"  Object : {bold(request.responsible_object_key)}"))
        _emit(yellow(f"  Action : {bold(request.directive_action)}"))

        while True:
            try:
                ans = input(yellow("  [operator] y / n > ")).strip().lower()
            except EOFError:
                ans = "n"
            if ans == "y":
                self._op_resp_q.put((True, "operator_confirmed"))
                _emit(green("  ✓ Confirmed\n"))
                return
            elif ans in ("n", "q", ""):
                self._op_resp_q.put((False, "operator_rejected"))
                _emit(red("  ✗ Rejected\n"))
                return
            _emit("  Please enter y or n.")


# ---------------------------------------------------------------------------
# Output helper — prints with a leading newline to avoid clobbering the
# prompt drawn by input() in the input thread
# ---------------------------------------------------------------------------
_print_lock = threading.Lock()


def _emit(text: str) -> None:
    with _print_lock:
        print(text, flush=True)


# ---------------------------------------------------------------------------
# Input thread
# ---------------------------------------------------------------------------

def _input_thread(cmd_q: queue.Queue) -> None:
    """Runs in a daemon thread.  Reads stdin and feeds _cmd_q."""
    while True:
        try:
            line = input(bold("\n[nav] > "))
        except EOFError:
            cmd_q.put(None)
            return
        except KeyboardInterrupt:
            # Ctrl-C: cancel current navigation and stay in the loop
            cmd_q.put("\x03")   # sentinel — controller will cancel, not exit
            continue
        stripped = line.strip()
        if stripped:
            cmd_q.put(stripped)


# ---------------------------------------------------------------------------
# Command resolver
# ---------------------------------------------------------------------------

def _resolve(node: NavigationTerminal, raw: str) -> Optional[str]:
    """
    Turn raw user input into a query string (object key or NL-parsed tag).
    Returns None if the command should be skipped.
    """
    if _looks_like_object_key(raw):
        _emit(cyan(f"  → Direct key: {bold(raw)}"))
        return raw

    _emit(cyan(f"  → Parsing NL: \"{raw}\" …"))
    return node.parse_nl(raw)


# ---------------------------------------------------------------------------
# Controller loop  (main thread)
# ---------------------------------------------------------------------------

def _controller(node: NavigationTerminal, cmd_q: queue.Queue) -> None:
    _emit(bold("\n╔══════════════════════════════════════════════════════╗"))
    _emit(bold( "║        Semantic Navigation Terminal                  ║"))
    _emit(bold( "╚══════════════════════════════════════════════════════╝"))
    _emit(dim("  Type an object key (chair:2) or NL command (I am tired)."))
    _emit(dim("  Type a new command at any time to cancel and reroute."))
    _emit(dim("  Ctrl-C cancels active navigation.  Ctrl-D exits.\n"))

    while True:
        raw = cmd_q.get()          # block until input_thread has something

        # Ctrl-D / explicit None → exit
        if raw is None:
            _emit(dim("\nExiting terminal."))
            return

        # Ctrl-C with nothing active → just show a newline
        if raw == "\x03":
            continue

        query = _resolve(node, raw)
        if query is None:
            continue

        # Navigate — loop handles preemption inline
        while query is not None:
            _emit(cyan(f"\n  → Navigating to {bold(query)} …"))
            result = node.navigate(query, raw if not _looks_like_object_key(raw) else "", cmd_q)

            if result.exit_requested:
                return

            if result.preempt_cmd is not None:
                preempt = result.preempt_cmd
                # Ctrl-C during navigation: cancel but stay in outer loop
                if preempt == "\x03":
                    _emit(yellow("  ↩ Navigation cancelled."))
                    query = None
                    continue

                # New destination typed: resolve it and loop back immediately
                raw = preempt
                query = _resolve(node, raw)
                continue

            # Navigation finished
            if result.success:
                _emit(green(f"\n  ✓ SUCCESS — reached {bold(query)}"))
            else:
                _emit(red(f"\n  ✗ FAILED — {result.outcome}: {result.failure_reason}"))

            query = None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationTerminal()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    cmd_q: queue.Queue = queue.Queue()
    inp_thread = threading.Thread(target=_input_thread, args=(cmd_q,), daemon=True)
    inp_thread.start()

    try:
        _controller(node, cmd_q)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
