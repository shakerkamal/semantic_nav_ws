#!/usr/bin/env python3
"""Offline metrics parser for the up-front recovery ablation (no ROS).

Reads eval/logs/upfront_*.log (run_upfront_trial.sh format) and writes
eval/upfront_ablation_results.csv. Modelled on eval/enroute_ablation.py but
simpler: the blocker is present from the start, so there is no mid-drive
trigger to correlate.

Two fields stay separate on purpose: raw_directive (what the model proposed)
and accepted_directive (what the filter admitted) — collapsing them makes the
containment claim unmeasurable. There is deliberately NO barrier_clear column:
the up-front clearance gate is a known near-no-op and must never be cited.

Usage:
  python3 eval/upfront_ablation.py            # all eval/logs/upfront_*.log
  python3 eval/upfront_ablation.py <log> ...  # explicit files
"""
import csv
import glob
import os
import re
import sys

import yaml

EVAL_DIR = os.path.dirname(os.path.abspath(__file__))

STAMP = re.compile(r"\[(\d+)\.(\d+)\]")
TRIAL = re.compile(r"\[TRIAL\] scenario=(\S+) arm=(\S+) rep=(\d+)")
DISPATCH_WALL = re.compile(r"\[TRIAL\] dispatch_wall=([0-9.]+)")
FINISH_WALL = re.compile(r"\[TRIAL\] finish_wall=([0-9.]+)")

PREFLIGHT_FAIL = re.compile(
    r"\[UP_FRONT\] Pre-flight validation failed")
ATTEMPT = re.compile(
    r"\[UP_FRONT\] attempt=(\d+) diagnosis=(\S+)"
    r" centroid=\(([-0-9.e]+), ([-0-9.e]+)\)")
AFFORDANCE = re.compile(
    r"\[AFFORDANCE\] tag='[^']*' -> openable=(True|False)"
    r" clearable=(True|False) safety=(\S+) confidence=(\d+)")
ELIGIBLE = re.compile(
    r"\[UP_FRONT\] eligible=\[([^\]]*)\] llm='([^']*)' -> directive=(\S+)"
    r" \(overridden=(True|False) reason=(\S+)\) responsible_tag='([^']*)'"
    r" match_type=(\S+) has_standoff=\S+ dist_to_barrier=([0-9.]+)m"
    r" within_verify_range=(True|False)")
LLM_LATENCY = re.compile(r"\[UP_FRONT\] LLM recovery response in ([0-9.]+)s")
# The allowed= request line is logged BEFORE the LLM call, so it survives the
# key-log buffer race that can truncate the trailing eligible= line. Only
# failure_stage='validation' invocations belong to the up-front lane (the
# navigator logs the same WARN for en-route Tier-3 calls).
ALLOWED = re.compile(
    r"\[UP_FRONT\] Requesting LLM recovery choice via \S+ \(allowed=\[([^\]]*)\]")
LLM_INVOKED = re.compile(
    r"\[RECOVERY\] LLM recovery invoked\..*failure_stage='validation'")
RECHECK = re.compile(
    r"\[UP_FRONT\] Recheck poll=\d+: barrier=\S+ plan_ok=(True|False)"
    r" barrier_ok=(True|False)")
OPERATOR = re.compile(
    r"\[UP_FRONT\] Operator decision for '[^']*': acknowledged=True")
REVALIDATED = re.compile(r"\[UP_FRONT\] Goal reachable")
ESCALATED = re.compile(
    r"\[UP_FRONT\] Directive '[^']*' needs operator action; escalating")
FINISHED = re.compile(
    r"\[EXECUTION\] Executor finished with status=\S+, success=(True|False),"
    r" object_key='([^']*)', db_version=(\d+)")
DISPATCH = re.compile(
    r"\[EXECUTION\] Sending goal to execute_pose.*?db_version=(\d+)")
RESPONSE = re.compile(
    r"NavigateToQuery_Response\(success=(True|False), outcome='([^']*)'"
    r".*?reached_target='([^']*)'")
ENROUTE_FIRED = re.compile(r"\[RECOVERY/BT\] BT proposal response")


def _stamp(line):
    m = STAMP.search(line)
    if not m:
        return None
    return int(m.group(1)) + int(m.group(2)) / 1e9


def _eligible_list(group):
    return ";".join(
        part.strip().strip("'\"") for part in group.split(",") if part.strip())


def parse_trial(text: str, expected_goal_key: str = "bed:120",
                expected_final_directive: str = "",
                llm_latency_ref_s=None) -> dict:
    meta = TRIAL.search(text)
    if not meta:
        raise ValueError("no [TRIAL] marker")
    scenario, arm, rep = meta.groups()

    t_first = t_last = None
    attempts = []
    affordance = None
    eligibles = []
    alloweds = []
    llm_invocations = 0
    latencies = []
    recheck_plan_oks = []
    operator_done = False
    revalidated = False
    escalated = False
    standoff_reached = False
    goal_exec = None       # (success, object_key, db_version) of last non-standoff
    dispatch_dbv = ""
    enroute_fired = False

    for ln in text.splitlines():
        s = _stamp(ln)
        if s is not None:
            if t_first is None:
                t_first = s
            t_last = s
        m = ATTEMPT.search(ln)
        if m:
            attempts.append(m.groups())
        m = AFFORDANCE.search(ln)
        if m and affordance is None:
            affordance = m.groups()
        m = ELIGIBLE.search(ln)
        if m:
            eligibles.append(m.groups())
        m = ALLOWED.search(ln)
        if m:
            alloweds.append(m.group(1))
        if LLM_INVOKED.search(ln):
            llm_invocations += 1
        m = LLM_LATENCY.search(ln)
        if m:
            latencies.append(float(m.group(1)))
        m = RECHECK.search(ln)
        if m:
            recheck_plan_oks.append(
                (m.group(1) == "True", m.group(2) == "True"))
        if OPERATOR.search(ln):
            operator_done = True
        if REVALIDATED.search(ln):
            revalidated = True
        if ESCALATED.search(ln):
            escalated = True
        m = FINISHED.search(ln)
        if m:
            if m.group(2) == "__standoff__":
                if m.group(1) == "True":
                    standoff_reached = True
            else:
                goal_exec = (m.group(1) == "True", m.group(2), m.group(3))
        m = DISPATCH.search(ln)
        if m and not dispatch_dbv:
            dispatch_dbv = m.group(1)
        if ENROUTE_FIRED.search(ln):
            enroute_fired = True

    triggered = PREFLIGHT_FAIL.search(text) is not None

    # Terminal outcome: the service response is authoritative when present;
    # then the final goal-key Executor-finished line; an escalation with
    # neither is the operator-handoff case.
    # reached_target echoes the QUERY STRING (the parsed tag, e.g. 'bed') when
    # no redirect happened — only a redirect substitutes an object key. So the
    # original counts as reached when the field is empty, the key, or the
    # key's tag.
    goal_tag = expected_goal_key.rsplit(":", 1)[0]
    resp = RESPONSE.search(text)
    if resp is not None:
        code = resp.group(2)
        if code == "REACHED":
            reached = resp.group(3)
            outcome = ("original-target-reached"
                       if reached in ("", expected_goal_key, goal_tag)
                       else "intent-preserving-alternative")
        elif code == "NEEDS_OPERATOR":
            outcome = "needs-operator"
        elif code == "EXECUTION_FAILED":
            outcome = "aborted"
        else:
            outcome = code.lower().replace("_", "-")
    elif goal_exec is not None:
        outcome = "original-target-reached" if goal_exec[0] else "aborted"
    elif escalated:
        outcome = "needs-operator"
    else:
        outcome = "aborted"
    nav_success = outcome in (
        "original-target-reached", "intent-preserving-alternative")
    # A triggered run whose response is NEEDS_OPERATOR escalated by
    # definition, even when the explicit escalation line fell to the slice.
    if triggered and outcome == "needs-operator":
        escalated = True

    preserved = bool(
        goal_exec is not None and goal_exec[0]
        and goal_exec[1] == expected_goal_key)
    if not preserved and resp is not None and resp.group(2) == "REACHED":
        preserved = resp.group(3) in ("", expected_goal_key, goal_tag)

    raw_seq = ";".join(e[1] for e in eligibles)
    accepted_seq = ";".join(e[2] for e in eligibles)
    final_directive = eligibles[-1][2] if eligibles else ""
    last = eligibles[-1] if eligibles else None

    directive_correct = ((final_directive == expected_final_directive)
                         if expected_final_directive else "")

    dw = DISPATCH_WALL.search(text)
    fw = FINISH_WALL.search(text)
    resolution = ""
    if dw is not None and fw is not None:
        resolution = round(float(fw.group(1)) - float(dw.group(1)), 3)
    elif t_first is not None and t_last is not None:
        resolution = round(t_last - t_first, 3)

    total_latency = round(sum(latencies), 3) if latencies else ""
    mean_latency = (round(sum(latencies) / len(latencies), 3)
                    if latencies else "")

    # LLM-serving-hardware decomposition. The up-front campaign's LLM ran
    # locally (uni server down) at ~25-40s/call vs the en-route grid's ~1.3s;
    # measured columns stay measured, and comparability comes from the
    # remainder with LLM time removed plus a clearly-labelled normalized
    # estimate that substitutes the declared en-route reference latency.
    minus_llm = ""
    norm_llm = ""
    if resolution != "":
        minus_llm = round(resolution - (sum(latencies) if latencies else 0), 3)
        if llm_latency_ref_s is not None:
            norm_llm = round(
                minus_llm + len(latencies) * float(llm_latency_ref_s), 3)

    return {
        "scenario": scenario,
        "arm": arm,
        "rep": int(rep),
        # U0 short-circuits pre-flight validation entirely (the enabled flag
        # is checked first), so "not triggered" would misreport a check that
        # never ran as a pass.
        "initial_path_valid": "" if arm == "U0" else not triggered,
        "upfront_recovery_triggered": triggered,
        "attempts": len(attempts),
        "diagnosis": attempts[0][1] if attempts else "",
        "barrier_centroid_x": float(attempts[0][2]) if attempts else "",
        "barrier_centroid_y": float(attempts[0][3]) if attempts else "",
        "affordance_inferred": affordance is not None,
        "inf_openable": (affordance[0] == "True") if affordance else "",
        "inf_clearable": (affordance[1] == "True") if affordance else "",
        "inf_safety": affordance[2] if affordance else "",
        "affordance_confidence": int(affordance[3]) if affordance else "",
        "eligible_actions": _eligible_list(last[0]) if last else "",
        "open_door_ever_eligible": any(
            "open_door_then_replan" in e[0] for e in eligibles) or any(
            "open_door_then_replan" in a for a in alloweds),
        "raw_directive": raw_seq,
        "accepted_directive": accepted_seq,
        "final_directive": final_directive,
        "directive_correct": directive_correct,
        "proposal_overridden": any(e[3] == "True" for e in eligibles),
        "override_reason": last[4] if last else "",
        "responsible_tag": last[5] if last else "",
        "responsible_match_type": last[6] if last else "",
        "dist_to_barrier_m": float(last[7]) if last else "",
        "within_verify_range": (last[8] == "True") if last else "",
        "llm_calls": llm_invocations or len(latencies),
        "llm_latency_s_total": total_latency,
        "llm_latency_s_mean": mean_latency,
        "standoff_reached": standoff_reached,
        "recheck_polls": len(recheck_plan_oks),
        "plan_ok_final": recheck_plan_oks[-1][0] if recheck_plan_oks else "",
        # True when the final poll had a VALID plan that the barrier-clear
        # gate vetoed (plan_ok=True, barrier_ok=False) — the r5 failure
        # mechanism. Citable as a failure cause, never as clearance evidence.
        "barrier_gate_vetoed": (
            (recheck_plan_oks[-1][0] and not recheck_plan_oks[-1][1])
            if recheck_plan_oks else ""),
        "operator_action_completed": operator_done,
        "original_goal_revalidated": revalidated,
        "escalated": escalated,
        "enroute_recovery_fired": enroute_fired,
        "navigation_success": nav_success,
        "terminal_outcome": outcome,
        "original_target_preserved": preserved,
        "time_to_resolution_s": resolution,
        "time_to_resolution_minus_llm_s": minus_llm,
        "time_to_resolution_norm_llm_s": norm_llm,
        "db_version": (goal_exec[2] if goal_exec else "") or dispatch_dbv,
    }


def main() -> None:
    with open(os.path.join(EVAL_DIR, "upfront_scenarios.yaml")) as f:
        cfg = yaml.safe_load(f)
    goal_key = cfg["common"]["goal_object_key"]
    scenarios = cfg["scenarios"]
    paths = sys.argv[1:] or sorted(
        glob.glob(os.path.join(EVAL_DIR, "logs", "upfront_*.log")))
    rows = []
    for path in paths:
        text = open(path).read()
        meta = TRIAL.search(text)
        if not meta:
            continue  # session/*.key.log files also match the glob
        scenario, arm = meta.group(1), meta.group(2)
        if scenario not in scenarios:
            continue
        expected = scenarios[scenario]["expected_final_directive"].get(arm, "")
        rows.append(parse_trial(text, goal_key, expected,
                                cfg["common"].get("llm_latency_ref_s")))
    if not rows:
        print("no upfront trial logs found")
        return
    out = os.path.join(EVAL_DIR, "upfront_ablation_results.csv")
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {out} ({len(rows)} rows)")


if __name__ == "__main__":
    main()
