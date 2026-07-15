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
PROPOSAL = re.compile(
    r"\[RECOVERY/BT\] BT proposal response: success=\S+, action='([^']*)'"
    r"(?:, target_object_tag='([^']*)')?")
LLM_INVOKED = re.compile(r"\[RECOVERY\] LLM recovery invoked")
REDIRECT = re.compile(
    r"Retry target redirected from blocked '[^']*' to reachable alternative"
    r" '[^']*' \(tag='([^']*)'\)")
BACKUP = re.compile(r"Running backup")
DETECT = re.compile(r"\[MOCK_DETECTOR\] dist=([0-9.]+)")

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


def parse_trial(text: str, expected_directive: str) -> dict:
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

    # Prefer the Executor-finished stamp; fall back to the last stamped line
    # for runs that terminate via the service (NEEDS_OPERATOR etc.).
    end_t = t_finish if t_finish is not None else t_last
    resolution = ""
    if t_dispatch is not None and end_t is not None:
        resolution = round(end_t - t_dispatch, 3)

    return {
        "scenario": scenario,
        "variant": variant,
        "rep": int(rep),
        "terminal_outcome": outcome,
        "resolving_tier": tier,
        "directive_chosen": directive,
        "directive_correct": (directive == expected_directive)
                             if directive != "none" or expected_directive == "none"
                             else False,
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
        gt = {name: sc["expected_directive"]
              for name, sc in yaml.safe_load(f)["scenarios"].items()}
    paths = sys.argv[1:] or sorted(
        glob.glob(os.path.join(EVAL_DIR, "logs", "enroute_*.log")))
    rows = []
    for path in paths:
        text = open(path).read()
        scenario = TRIAL.search(text).group(1)
        rows.append(parse_trial(text, gt[scenario]))
    out = os.path.join(EVAL_DIR, "enroute_ablation_results.csv")
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {out} ({len(rows)} rows)")


if __name__ == "__main__":
    main()
