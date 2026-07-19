#!/usr/bin/env python3
"""Offline metrics parser for the en-route ablation (no ROS).

Reads eval/logs/enroute_*.log (run_enroute_trial.sh format) and writes
eval/enroute_ablation_results.csv (spec section 6 columns).

Usage:
  python3 eval/enroute_ablation.py            # all eval/logs/enroute_*.log
  python3 eval/enroute_ablation.py <log> ...  # explicit files
"""
import csv
import glob
import os
import re
import sys

import yaml

EVAL_DIR = os.path.dirname(os.path.abspath(__file__))

STAMP = re.compile(r"\[(\d+)\.(\d+)\]")
TRIAL = re.compile(
    r"\[TRIAL\] scenario=(\S+) variant=(\S+) rep=(\d+) commit=(\S+)")
DISPATCH = re.compile(
    r"\[EXECUTION\] Sending goal to execute_pose.*?db_version=(\d+)")
FINISHED = re.compile(
    r"\[EXECUTION\] Executor finished with status=\S+, success=(True|False),"
    r" object_key='([^']*)', db_version=(\d+)")
# Authoritative terminal signal for runs that end without an Executor-finished
# line (recovery exhausted -> NEEDS_OPERATOR, RESOLUTION_FAILED, ...): the
# NavigateToQuery service response captured by the run wrapper.
RESPONSE = re.compile(
    r"NavigateToQuery_Response\(success=(True|False), outcome='([^']*)'")
# Wall-clock markers the run wrapper stamps around the service call — the
# reliable timing source when a silent successful drive lets the buffered
# Executor-finished line get dropped from the log slice.
DISPATCH_WALL = re.compile(r"\[TRIAL\] dispatch_wall=([0-9.]+)")
FINISH_WALL = re.compile(r"\[TRIAL\] finish_wall=([0-9.]+)")
PROPOSAL = re.compile(
    r"\[RECOVERY/BT\] BT proposal response: success=\S+, action='([^']*)'"
    r"(?:, target_object_tag='([^']*)')?")
LLM_INVOKED = re.compile(r"\[RECOVERY\] LLM recovery invoked")
REDIRECT = re.compile(
    r"Retry target redirected from blocked '[^']*' to reachable alternative"
    r" '[^']*' \(tag='([^']*)'\)")
BACKUP = re.compile(r"Running backup")
DETECT = re.compile(r"\[MOCK_DETECTOR\] dist=([0-9.]+)")

# Semantic-branch evidence markers. A final REACHED response is NOT proof the
# semantic recovery branch succeeded -- the outer Nav2 fallback (backup/replan)
# can reach the goal after WaitForBarrierClear/departure failed. These let the
# parser separate a genuine end-to-end semantic-recovery success from a
# navigation success that a geometric fallback rescued.
MATCH_OBJECT = re.compile(
    r"/match_responsible_object: candidates=\d+, success=(True|False),"
    r" match_type='([^']*)', responsible_object_key='([^']*)'")
OPERATOR_DONE = re.compile(r"OperatorPrompt\] action_completed token='([^']*)'")
DEPARTURE_OK = re.compile(
    r"\[WaitForDynamicObstacleDeparture\].*?blocking=false"
    r".*?clear_streak=(\d+)/(\d+)")
CLEANUP_DONE = re.compile(
    r"\[WaitForBarrierClear\] cleanup_local_grids completed modified=(-?\d+)")
BARRIER_OK = re.compile(
    r"\[WaitForBarrierClear\] barrier clear and stabilized")

# Directives whose semantic branch funnels through an intervention + the
# WaitForBarrierClear gate (so both must succeed for a real semantic recovery).
BARRIER_DIRECTIVES = frozenset(
    {"open_door_then_replan", "clear_object_then_replan", "wait_then_replan"})

REAPPROACH_NEAR_M = 1.5


def _stamp(line):
    m = STAMP.search(line)
    if not m:
        return None
    return int(m.group(1)) + int(m.group(2)) / 1e9


def _reapproach_count(dists, near=REAPPROACH_NEAR_M):
    count, was_far = 0, True
    for d in dists:
        if was_far and d < near:
            count += 1
            was_far = False
        elif d >= near:
            was_far = True
    return count


def parse_trial(
    text: str, expected_directive: str, expected_object: str = "") -> dict:
    lines = text.splitlines()
    meta = TRIAL.search(text)
    if not meta:
        raise ValueError("no [TRIAL] marker")
    scenario, variant, rep, commit = meta.groups()

    t_dispatch = t_finish = t_last = None
    success = None
    db_version = ""
    dispatch_dbv = ""
    directives = []
    target_tag = ""
    redirected_tag = ""
    llm_invocations = []
    llm_responses = []
    backups = 0
    dists = []
    # Semantic-branch evidence (see marker regexes above).
    saw_match = False
    resp_key = ""
    resp_match_type = ""
    operator_done = False
    departure_confirmed = False
    barrier_succeeded = False
    cleanup_invoked = False
    cleanup_count = ""

    for ln in lines:
        s = _stamp(ln)
        if s is not None and (t_last is None or s > t_last):
            t_last = s
        m = DISPATCH.search(ln)
        if m and t_dispatch is None:
            t_dispatch = _stamp(ln)
            dispatch_dbv = m.group(1)
        m = FINISHED.search(ln)
        if m:
            success = (m.group(1) == "True")
            db_version = m.group(3)
            t_finish = _stamp(ln)
        m = PROPOSAL.search(ln)
        if m:
            directives.append(m.group(1))
            if m.group(2):
                target_tag = m.group(2)
            llm_responses.append(_stamp(ln))
        if LLM_INVOKED.search(ln):
            llm_invocations.append(_stamp(ln))
        m = REDIRECT.search(ln)
        if m:
            redirected_tag = m.group(1)
        if BACKUP.search(ln):
            backups += 1
        m = DETECT.search(ln)
        if m:
            dists.append(float(m.group(1)))
        m = MATCH_OBJECT.search(ln)
        if m:
            saw_match = True
            if m.group(1) == "True":
                resp_match_type = m.group(2)
                resp_key = m.group(3)
        if OPERATOR_DONE.search(ln):
            operator_done = True
        m = DEPARTURE_OK.search(ln)
        if m and m.group(1) == m.group(2):
            departure_confirmed = True
        m = CLEANUP_DONE.search(ln)
        if m:
            cleanup_invoked = True
            cleanup_count = int(m.group(1))
        if BARRIER_OK.search(ln):
            barrier_succeeded = True

    directive = directives[0] if directives else "none"
    if directives:
        tier = "T3"
    elif backups:
        tier = "T2"
    else:
        tier = "none"

    # The service response is authoritative when present; the Executor-finished
    # success flag is the fallback (used by the fixtures and any run where the
    # wrapper did not capture the response line).
    resp = RESPONSE.search(text)
    if resp is not None:
        code = resp.group(2)
        if code == "REACHED":
            outcome = ("intent-preserving-alternative" if redirected_tag
                       else "original-target-reached")
        elif code == "NEEDS_OPERATOR":
            outcome = "needs-operator"
        elif code == "EXECUTION_FAILED":
            outcome = "aborted"
        else:
            outcome = code.lower().replace("_", "-")
    elif success and redirected_tag:
        outcome = "intent-preserving-alternative"
    elif success:
        outcome = "original-target-reached"
    else:
        outcome = "aborted"

    latency = ""
    if llm_invocations and llm_responses:
        latency = round(llm_responses[0] - llm_invocations[0], 3)

    # Resolution time, most-accurate source first:
    #  1. internal ROS timing (Executor-finished stamp - dispatch stamp);
    #  2. the wrapper's wall-clock markers (survive the buffer race that drops
    #     the Executor-finished line on a silent successful drive);
    #  3. last stamped line - dispatch (recovery-exhausted runs keep logging to
    #     the end, so their last stamp is a fair proxy).
    dw = DISPATCH_WALL.search(text)
    fw = FINISH_WALL.search(text)
    resolution = ""
    if t_dispatch is not None and t_finish is not None:
        resolution = round(t_finish - t_dispatch, 3)
    elif dw is not None and fw is not None:
        resolution = round(float(fw.group(1)) - float(dw.group(1)), 3)
    elif t_dispatch is not None and t_last is not None:
        resolution = round(t_last - t_dispatch, 3)

    directive_correct = ((directive == expected_directive)
                         if directive != "none" or expected_directive == "none"
                         else False)
    nav_success = outcome in (
        "original-target-reached", "intent-preserving-alternative")
    directive_issued = bool(directives)
    semantic_triggered = saw_match or directive_issued or bool(llm_invocations)

    responsible_object_correct = (
        (resp_key == expected_object) if expected_object else "")

    requires_barrier = expected_directive in BARRIER_DIRECTIVES
    intervention_ok = (
        (operator_done or departure_confirmed) if requires_barrier else True)
    if requires_barrier:
        branch_completed = barrier_succeeded
    elif expected_directive == "retry_target":
        branch_completed = bool(redirected_tag) and nav_success
    else:
        branch_completed = ""  # not applicable (e.g. S1 transient control)

    # The burning-issue flag: the semantic branch failed but navigation still
    # reached the goal, i.e. the outer geometric fallback rescued it.
    outer_fallback = bool(
        directive_issued and nav_success and branch_completed is False)

    # End-to-end semantic-recovery success requires EVERY link, not a final
    # REACHED alone: correct object AND correct directive AND the required
    # intervention/departure AND the barrier gate AND navigation reached target.
    if expected_directive == "none":
        semantic_recovery_success = ""  # S1 tests that NO recovery was needed
    else:
        object_ok = responsible_object_correct if expected_object else True
        barrier_ok = barrier_succeeded if requires_barrier else True
        semantic_recovery_success = bool(
            nav_success and directive_correct and object_ok
            and intervention_ok and barrier_ok)

    return {
        "scenario": scenario,
        "variant": variant,
        "rep": int(rep),
        "terminal_outcome": outcome,
        "resolving_tier": tier,
        "semantic_recovery_triggered": semantic_triggered,
        "responsible_object_key": resp_key,
        "responsible_match_type": resp_match_type,
        "responsible_object_correct": responsible_object_correct,
        "directive_issued": directive_issued,
        "directive_chosen": directive,
        "directive_correct": directive_correct,
        "operator_action_completed": operator_done,
        "departure_confirmed": departure_confirmed,
        "cleanup_invoked": cleanup_invoked,
        "cleanup_modified_count": cleanup_count,
        "barrier_clear_succeeded": barrier_succeeded,
        "semantic_branch_completed": branch_completed,
        "outer_fallback_after_semantic_failure": outer_fallback,
        "navigation_success": nav_success,
        "semantic_recovery_success": semantic_recovery_success,
        "target_object_tag": redirected_tag or target_tag,
        "recovery_cycles": backups,
        "llm_calls": len(llm_invocations),
        "llm_latency_s": latency,
        "time_to_resolution_s": resolution,
        "reapproach_count": _reapproach_count(dists) if dists else "",
        "min_standoff_m": min(dists) if dists else "",
        "db_version": db_version or dispatch_dbv,
        "code_commit": commit,
    }


def main() -> None:
    with open(os.path.join(EVAL_DIR, "enroute_scenarios.yaml")) as f:
        scenarios = yaml.safe_load(f)["scenarios"]
    gt = {name: sc["expected_directive"] for name, sc in scenarios.items()}
    gt_obj = {name: (sc.get("detector") or {}).get("object_key", "") or ""
              for name, sc in scenarios.items()}
    paths = sys.argv[1:] or sorted(
        glob.glob(os.path.join(EVAL_DIR, "logs", "enroute_*.log")))
    rows = []
    for path in paths:
        text = open(path).read()
        scenario = TRIAL.search(text).group(1)
        rows.append(parse_trial(text, gt[scenario], gt_obj.get(scenario, "")))
    out = os.path.join(EVAL_DIR, "enroute_ablation_results.csv")
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {out} ({len(rows)} rows)")


if __name__ == "__main__":
    main()
