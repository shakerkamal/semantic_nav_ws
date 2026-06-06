import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class LlamaActionClient:
    """Thin synchronous wrapper around llama_msgs/GenerateResponse.

    Accepts any object that exposes wait_for_server / send_goal_async so unit
    tests can pass a fake. In production, callers pass an rclpy ActionClient.

    Two operating modes:

    executor_is_running=False (default, eval harness):
        No executor is spinning. _spin_until_done calls
        rclpy.spin_until_future_complete(node, future) to drive the event loop
        from this thread. node must be set.

    executor_is_running=True (resolver_node with MultiThreadedExecutor):
        The executor is already running its own thread pool. Calling
        spin_until_future_complete here would add the node to a second executor
        causing double-dispatch. Instead _spin_until_done just polls future.done()
        while the executor's threads deliver the action response concurrently.
        The service callback and the ActionClient must each have a
        ReentrantCallbackGroup so they can run concurrently.
    """

    action_client: object
    logger: object
    node: object = None               # rclpy Node; required for real ROS calls
    executor_is_running: bool = False  # True when called from a MultiThreadedExecutor callback
    wait_timeout_sec: float = 60.0
    send_timeout_sec: float = 120.0
    result_timeout_sec: float = 300.0

    def call(
        self,
        prompt: str,
        gbnf_grammar: str,
        max_tokens: int = 128,
        temperature: float = 0.0,
        top_k: int = 1,
        top_p: float = 1.0,
        reset: bool = True,
    ) -> Optional[str]:
        if not self.action_client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.logger.error(
                f"LLM action server unavailable after {self.wait_timeout_sec}s"
            )
            return None

        try:
            from llama_msgs.action import GenerateResponse
            goal = GenerateResponse.Goal()
        except Exception:
            class _G:
                class _SC:
                    pass
                sampling_config = _SC()
                prompt = ""
                reset = True
                stop = []
            goal = _G()

        goal.prompt = prompt
        if hasattr(goal, "reset"):
            goal.reset = bool(reset)
        if hasattr(goal, "stop"):
            goal.stop = []

        sc = goal.sampling_config
        for name, val in (("temp", temperature), ("top_k", top_k), ("top_p", top_p)):
            if hasattr(sc, name):
                setattr(sc, name, val)
        for name in ("n_predict", "max_tokens", "max_new_tokens"):
            if hasattr(sc, name):
                setattr(sc, name, int(max_tokens))
                break
        if hasattr(sc, "grammar"):
            sc.grammar = gbnf_grammar
        else:
            self.logger.error("SamplingConfig has no 'grammar' field; cannot enforce GBNF.")
            return None

        send_future = self.action_client.send_goal_async(goal)
        if not self._spin_until_done(send_future, self.send_timeout_sec):
            self.logger.error("Timed out sending LLM goal")
            return None
        goal_handle = send_future.result()
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            self.logger.error("LLM goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        if not self._spin_until_done(result_future, self.result_timeout_sec):
            self.logger.error("Timed out waiting for LLM result")
            return None

        wrap = result_future.result()
        if wrap is None or getattr(wrap, "status", 0) != 4:   # STATUS_SUCCEEDED
            self.logger.error(
                f"LLM finished with status={getattr(wrap, 'status', None)}"
            )
            return None
        text = self._extract_text(wrap.result)
        return text.strip() if text else None

    def _spin_until_done(self, future, timeout_sec: float) -> bool:
        if self.executor_is_running:
            # A MultiThreadedExecutor is already running in another thread.
            # Calling spin_until_future_complete here would add the node to a
            # second executor causing double-dispatch. Just poll: the executor's
            # threads will deliver the action response and complete the future.
            deadline = time.monotonic() + timeout_sec
            while not future.done():
                if time.monotonic() >= deadline:
                    return False
                time.sleep(0.02)
            return True

        if self.node is not None:
            # No executor is running (eval harness). Drive the event loop here.
            try:
                import rclpy
                rclpy.spin_until_future_complete(
                    self.node, future, timeout_sec=timeout_sec
                )
                return future.done()
            except Exception:
                pass

        # Pure polling fallback (unit tests with fake clients).
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)
        return True

    @staticmethod
    def _extract_text(result) -> str:
        if result is None:
            return ""
        if hasattr(result, "response"):
            r = result.response
            if hasattr(r, "text"):
                return str(r.text)
            return str(r)
        for f in ("text", "output"):
            if hasattr(result, f):
                return str(getattr(result, f))
        return ""
