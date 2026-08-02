"""Tests for the up-front ablation harness (eval/upfront_ablation.py).

The parser is written against the four reference traces from the demonstration
runs (eval/open_set_A1.txt, eval/open_set_A2_<model>.txt), exactly as the
evaluation plan prescribes: they cover the two escalation flavours (A1 =
open_door never eligible; 8B = eligible but never chosen) and the success path
(14B, 32B). Real trial logs get a [TRIAL] header from run_upfront_trial.sh;
the fixtures are wrapped with a synthetic one here.
"""
import os
import subprocess
import sys

import yaml

EVAL_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, EVAL_DIR)

SCENARIOS_PATH = os.path.join(EVAL_DIR, "upfront_scenarios.yaml")
RUNNER_PATH = os.path.join(EVAL_DIR, "run_upfront_trial.sh")


def _fixture(name, scenario="U-B", arm="U2", rep=1, first_episode_only=False):
    text = open(os.path.join(EVAL_DIR, name)).read()
    if first_episode_only:
        # open_set_A1.txt holds two concatenated episodes; a real trial log
        # holds exactly one, so slice at the second initial dispatch.
        lines = text.splitlines()
        marks = [i for i, ln in enumerate(lines)
                 if "reason=bt_led_initial_dispatch" in ln]
        assert len(marks) >= 2
        text = "\n".join(lines[:marks[1]])
    header = f"[TRIAL] scenario={scenario} arm={arm} rep={rep} commit=fixture start=0\n"
    return header + text


# ---------------------------------------------------------------- A1 (no
# affordance inference): open_door_then_replan is NEVER eligible, so the run
# escalates. This is the structural claim the U1 arm must make measurable.

def test_parse_a1_escalates_because_open_door_never_eligible():
    from upfront_ablation import parse_trial
    row = parse_trial(_fixture("open_set_A1.txt", arm="U1",
                               first_episode_only=True))
    assert row["scenario"] == "U-B"
    assert row["arm"] == "U1"
    assert row["rep"] == 1
    assert row["initial_path_valid"] is False
    assert row["upfront_recovery_triggered"] is True
    assert row["attempts"] == 2
    assert row["diagnosis"] == "blocked"
    assert abs(row["barrier_centroid_x"] - (-2.456)) < 0.01
    assert abs(row["barrier_centroid_y"] - (-1.409)) < 0.01
    assert row["affordance_inferred"] is False
    assert row["inf_openable"] == ""
    assert row["open_door_ever_eligible"] is False
    # raw vs accepted stay separate; here nothing was overridden.
    assert row["raw_directive"] == "approach_and_recheck;retry_target"
    assert row["accepted_directive"] == "approach_and_recheck;retry_target"
    assert row["final_directive"] == "retry_target"
    assert row["proposal_overridden"] is False
    assert row["override_reason"] == "llm_selected"
    assert row["responsible_tag"] == "room partition"
    assert row["responsible_match_type"] == "verified"
    assert abs(row["dist_to_barrier_m"] - 1.2) < 0.01
    assert row["within_verify_range"] is True
    assert row["llm_calls"] == 2
    assert abs(row["llm_latency_s_total"] - 5.9) < 0.01
    assert row["standoff_reached"] is True
    assert row["recheck_polls"] == 6
    assert row["plan_ok_final"] is False
    assert row["operator_action_completed"] is False
    assert row["original_goal_revalidated"] is False
    assert row["escalated"] is True
    assert row["enroute_recovery_fired"] is False
    assert row["navigation_success"] is False
    assert row["terminal_outcome"] == "needs-operator"
    assert row["original_target_preserved"] is False


# ---------------------------------------------------------------- A2 8B:
# affordance IS inferred and open_door IS eligible, but the model never picks
# the operator action -> escalation of the second flavour. The CSV must keep
# this distinguishable from A1's never-eligible escalation.

def test_parse_a2_8b_escalates_despite_open_door_eligible():
    from upfront_ablation import parse_trial
    row = parse_trial(_fixture("open_set_A2_llama3_8b.txt"))
    assert row["affordance_inferred"] is True
    assert row["inf_openable"] is True
    assert row["inf_clearable"] is False
    assert row["inf_safety"] == "none"
    assert row["affordance_confidence"] == 100
    assert row["open_door_ever_eligible"] is True
    assert row["raw_directive"] == "approach_and_recheck;retry_target"
    assert row["final_directive"] == "retry_target"
    assert row["escalated"] is True
    assert row["operator_action_completed"] is False
    assert abs(row["dist_to_barrier_m"] - 1.4) < 0.01
    assert row["within_verify_range"] is True
    assert row["terminal_outcome"] == "needs-operator"


# ---------------------------------------------------------------- A2 14B:
# the full success path. Also the one reference trace where an EN-ROUTE
# recovery fired after dispatch — the parser must flag that contamination.

def test_parse_a2_14b_full_success_path():
    from upfront_ablation import parse_trial
    row = parse_trial(_fixture("open_set_A2_qwen3_14b.txt"))
    assert row["initial_path_valid"] is False
    assert row["upfront_recovery_triggered"] is True
    assert row["attempts"] == 2
    assert row["affordance_inferred"] is True
    assert row["inf_openable"] is True
    assert row["inf_clearable"] is False
    assert row["affordance_confidence"] == 90
    assert row["open_door_ever_eligible"] is True
    assert row["raw_directive"] == "approach_and_recheck;open_door_then_replan"
    assert row["accepted_directive"] == (
        "approach_and_recheck;open_door_then_replan")
    assert row["final_directive"] == "open_door_then_replan"
    assert row["llm_calls"] == 2
    assert abs(row["llm_latency_s_total"] - 2.8) < 0.01
    assert row["standoff_reached"] is True
    assert row["recheck_polls"] == 10
    assert row["plan_ok_final"] is True
    assert row["operator_action_completed"] is True
    assert row["original_goal_revalidated"] is True
    assert row["escalated"] is False
    assert row["enroute_recovery_fired"] is True
    assert row["navigation_success"] is True
    assert row["terminal_outcome"] == "original-target-reached"
    assert row["original_target_preserved"] is True
    assert row["db_version"] == "1193208084"
    # whole-pipeline time from the log stamps (no wall markers in fixtures)
    assert abs(row["time_to_resolution_s"] - 158.9) < 0.5


def test_parse_a2_32b_full_success_path():
    from upfront_ablation import parse_trial
    row = parse_trial(_fixture("open_set_A2_qwen3_32b.txt"))
    assert row["affordance_inferred"] is True
    assert row["affordance_confidence"] == 90
    assert row["final_directive"] == "open_door_then_replan"
    assert row["llm_calls"] == 2
    assert abs(row["llm_latency_s_total"] - 5.5) < 0.01
    assert row["operator_action_completed"] is True
    assert row["original_goal_revalidated"] is True
    assert row["recheck_polls"] == 10
    assert row["plan_ok_final"] is True
    assert row["navigation_success"] is True
    assert row["original_target_preserved"] is True
    assert row["enroute_recovery_fired"] is False


# Real U-B/U1 r1 (2026-08-02) shape: the key-log slice was cut mid-second-LLM
# call (buffer race), losing the final eligible/escalation lines. The row must
# still carry the structural claim: the allowed= request line proves open_door
# was never offered, the invocation lines count both LLM calls, and the
# authoritative NEEDS_OPERATOR response marks the escalation.
FIXTURE_U1_TRUNCATED_TAIL = """\
[TRIAL] scenario=U-B arm=U1 rep=1 commit=deadbee start=1785677044
[TRIAL] dispatch_wall=1785677059.713889035
response:
semantic_nav_interfaces.srv.NavigateToQuery_Response(success=False, outcome='NEEDS_OPERATOR', failure_reason="Could not reach 'bed'.", reached_target='')
[TRIAL] finish_wall=1785677177.951856005
[navigation_orchestrator-25] [INFO] [1785677062.374958019] [navigation_orchestrator]: [UP_FRONT] Pre-flight validation failed; entering up-front blockage recovery.
[navigation_orchestrator-25] [INFO] [1785677069.448681926] [navigation_orchestrator]: [UP_FRONT] attempt=0 diagnosis=blocked centroid=(-2.473692662293575, -1.3167912140282807)
[navigator_node-24] [WARN] [1785677069.500000000] [navigator_node]: [RECOVERY] LLM recovery invoked. original_target='bed:120', failure_stage='validation', trigger_source='', match_type='verified', responsible_object_key='room partition:121', nav2_message='up-front global blockage', remaining_retry_budget=2
[navigation_orchestrator-25] [INFO] [1785677104.184340416] [navigation_orchestrator]: [UP_FRONT] LLM recovery response in 34.7s: success=True action='approach_and_recheck' rationale='...' message='LLM recovery chose approach_and_recheck.'
[navigation_orchestrator-25] [INFO] [1785677104.184844697] [navigation_orchestrator]: [UP_FRONT] eligible=['approach_and_recheck', 'retry_target', 'give_up'] llm='approach_and_recheck' -> directive=approach_and_recheck (overridden=False reason=llm_selected) responsible_tag='room partition' match_type=verified has_standoff=True dist_to_barrier=3.2m within_verify_range=False
[navigation_orchestrator-25] [INFO] [1785677112.280593579] [navigation_orchestrator]: [EXECUTION] Executor finished with status=SUCCEEDED(4), success=True, object_key='__standoff__', db_version=3498918824, db_stamp=1784033173.750330112, message='Navigation succeeded'
[navigation_orchestrator-25] [INFO] [1785677148.543056901] [navigation_orchestrator]: [UP_FRONT] Still blocked (barrier not confirmed clear) after approach + wait; re-diagnosing.
[navigation_orchestrator-25] [INFO] [1785677156.089463143] [navigation_orchestrator]: [UP_FRONT] attempt=1 diagnosis=blocked centroid=(-2.4716924616038356, -1.3642717714314883)
[navigation_orchestrator-25] [INFO] [1785677156.105415559] [navigation_orchestrator]: [UP_FRONT] Requesting LLM recovery choice via /propose_recovery (allowed=['retry_target', 'give_up'], timeout=45s).
[navigator_node-24] [WARN] [1785677156.107842268] [navigator_node]: [RECOVERY] LLM recovery invoked. original_target='bed:120', failure_stage='validation', trigger_source='', match_type='verified', responsible_object_key='room partition:121', nav2_message='up-front global blockage', remaining_retry_budget=2
"""


def test_parse_u1_truncated_tail_still_carries_the_structural_claim():
    from upfront_ablation import parse_trial
    row = parse_trial(FIXTURE_U1_TRUNCATED_TAIL)
    assert row["affordance_inferred"] is False
    # allowed= request lines are eligibility evidence too — the second
    # eligible= line was lost to the slice, the allowed= line was not.
    assert row["open_door_ever_eligible"] is False
    assert row["llm_calls"] == 2          # invocation lines, not latency lines
    assert row["llm_latency_s_total"] == 34.7   # only the captured call
    assert row["escalated"] is True       # triggered + NEEDS_OPERATOR response
    assert row["terminal_outcome"] == "needs-operator"
    assert row["navigation_success"] is False
    # An allowed= line that DOES offer open_door must flip the eligibility.
    offered = FIXTURE_U1_TRUNCATED_TAIL.replace(
        "allowed=['retry_target', 'give_up']",
        "allowed=['open_door_then_replan', 'retry_target', 'give_up']")
    assert parse_trial(offered)["open_door_ever_eligible"] is True


# ---------------------------------------------------------------- U-A control
# and the wrapper's authoritative response/wall markers.

FIXTURE_CONTROL = """\
[TRIAL] scenario=U-A arm=U2 rep=1 commit=deadbee start=1785600000
[TRIAL] dispatch_wall=1785600001.000000000
response:
semantic_nav_interfaces.srv.NavigateToQuery_Response(success=True, outcome='REACHED', failure_reason='', reached_target='bed:120')
[TRIAL] finish_wall=1785600091.500000000
[navigation_orchestrator-25] [INFO] [1785600001.5] [navigation_orchestrator]: [BT_LED] Pre-flight validation passed (or up-front recovery disabled); dispatching ExecutePose. The Nav2 BT owns further recovery.
[navigation_orchestrator-25] [INFO] [1785600090.0] [navigation_orchestrator]: [EXECUTION] Executor finished with status=SUCCEEDED(4), success=True, object_key='bed:120', db_version=42, db_stamp=1.0, message='Navigation succeeded'
"""


# Real U-B/U0 r1 (2026-08-02) shape: with up_front_recovery_enabled=false the
# orchestrator SKIPS pre-flight validation entirely (short-circuit) and
# dispatches into the geometric BT, which aborts. initial_path_valid must not
# read True for a check that never ran.
FIXTURE_U0_ABORT = """\
[TRIAL] scenario=U-B arm=U0 rep=1 commit=deadbee start=1785683776
[TRIAL] dispatch_wall=1785683782.874039831
response:
semantic_nav_interfaces.srv.NavigateToQuery_Response(success=False, outcome='NEEDS_OPERATOR', failure_reason="Could not reach 'bed'.", reached_target='')
[TRIAL] finish_wall=1785683791.830553467
[navigation_orchestrator-25] [INFO] [1785683783.535516478] [navigation_orchestrator]: [BT_LED] Pre-flight validation passed (or up-front recovery disabled); dispatching ExecutePose. The Nav2 BT owns further recovery.
[navigation_orchestrator-25] [INFO] [1785683783.536777307] [navigation_orchestrator]: [EXECUTION] Sending goal to execute_pose action server (object_key='bed:120', db_version=3498918824, db_stamp=1784033173.750330112): frame='map', x=-4.905, y=1.638
"""


def test_parse_u0_pre_flight_skipped_not_passed():
    from upfront_ablation import parse_trial
    row = parse_trial(FIXTURE_U0_ABORT)
    assert row["arm"] == "U0"
    assert row["initial_path_valid"] == ""   # skipped, not evaluated
    assert row["upfront_recovery_triggered"] is False
    assert row["escalated"] is False         # nothing up-front to escalate
    assert row["llm_calls"] == 0
    assert row["terminal_outcome"] == "needs-operator"
    assert row["navigation_success"] is False
    # U1/U2 keep the boolean semantics.
    assert parse_trial(FIXTURE_CONTROL)["initial_path_valid"] is True


def test_parse_control_no_trigger():
    from upfront_ablation import parse_trial
    row = parse_trial(FIXTURE_CONTROL)
    assert row["scenario"] == "U-A"
    assert row["arm"] == "U2"
    assert row["initial_path_valid"] is True
    assert row["upfront_recovery_triggered"] is False
    assert row["attempts"] == 0
    assert row["llm_calls"] == 0
    assert row["navigation_success"] is True
    assert row["terminal_outcome"] == "original-target-reached"
    assert row["original_target_preserved"] is True
    # wall markers beat the log stamps when present
    assert abs(row["time_to_resolution_s"] - 90.5) < 0.05


# Real U-B/U2 r1 (2026-08-01) shape: the run reached the ORIGINAL goal, but
# (a) reached_target echoes the query STRING ('bed', the parsed tag) — not the
# object key — because no redirect happened, and (b) the final goal-key
# Executor-finished line was lost to the key-log buffer race on a silent
# successful drive. The row must still read as an original-target success.
FIXTURE_REACHED_TAG_ONLY = """\
[TRIAL] scenario=U-B arm=U2 rep=1 commit=deadbee start=1785662001
[TRIAL] dispatch_wall=1785662017.635266873
response:
semantic_nav_interfaces.srv.NavigateToQuery_Response(success=True, outcome='REACHED', failure_reason='', reached_target='bed')
[TRIAL] finish_wall=1785662202.253332886
[navigation_orchestrator-25] [INFO] [1785662018.494845320] [navigation_orchestrator]: [UP_FRONT] Pre-flight validation failed; entering up-front blockage recovery.
[navigation_orchestrator-25] [INFO] [1785662077.873688756] [navigation_orchestrator]: [EXECUTION] Sending goal to execute_pose action server (object_key='__standoff__', db_version=3498918824, db_stamp=1784033173.750330112): frame='map', x=-1.556, y=-0.922
[navigation_orchestrator-25] [INFO] [1785662086.492340618] [navigation_orchestrator]: [EXECUTION] Executor finished with status=SUCCEEDED(4), success=True, object_key='__standoff__', db_version=3498918824, db_stamp=1784033173.750330112, message='Navigation succeeded'
[navigation_orchestrator-25] [INFO] [1785662187.418508117] [navigation_orchestrator]: [UP_FRONT] Goal reachable after operator 'open_door_then_replan'; dispatching.
"""


def test_parse_reached_target_may_be_the_query_tag_not_the_key():
    from upfront_ablation import parse_trial
    row = parse_trial(FIXTURE_REACHED_TAG_ONLY, expected_goal_key="bed:120")
    assert row["terminal_outcome"] == "original-target-reached"
    assert row["original_target_preserved"] is True
    assert row["navigation_success"] is True
    # goal executor line lost to the buffer race: db_version comes from the
    # dispatch line, timing from the wall markers.
    assert row["db_version"] == "3498918824"
    assert abs(row["time_to_resolution_s"] - 184.618) < 0.01
    # A genuine redirect (a real key that is neither the goal key nor its
    # tag) must still read as an alternative.
    redirected = FIXTURE_REACHED_TAG_ONLY.replace(
        "reached_target='bed'", "reached_target='tablet:116'")
    row = parse_trial(redirected, expected_goal_key="bed:120")
    assert row["terminal_outcome"] == "intent-preserving-alternative"
    assert row["original_target_preserved"] is False


def test_parse_needs_operator_response_is_authoritative():
    from upfront_ablation import parse_trial
    text = FIXTURE_CONTROL.replace(
        "success=True, outcome='REACHED'",
        "success=False, outcome='NEEDS_OPERATOR'")
    row = parse_trial(text)
    assert row["terminal_outcome"] == "needs-operator"
    assert row["navigation_success"] is False


def test_directive_correct_uses_expected_final_directive():
    from upfront_ablation import parse_trial
    row = parse_trial(_fixture("open_set_A2_qwen3_14b.txt"),
                      expected_final_directive="open_door_then_replan")
    assert row["directive_correct"] is True
    row = parse_trial(_fixture("open_set_A2_llama3_8b.txt"),
                      expected_final_directive="open_door_then_replan")
    assert row["directive_correct"] is False
    row = parse_trial(FIXTURE_CONTROL)
    assert row["directive_correct"] == ""


def test_llm_latency_decomposition_and_enroute_normalization():
    # The up-front campaign's LLM was served LOCALLY (uni server down), at
    # ~25-40s/call vs the en-route grid's ~1.3s. Measured values stay measured;
    # comparability comes from (a) time_to_resolution_minus_llm_s — the
    # hardware-independent remainder — and (b) time_to_resolution_norm_llm_s,
    # which substitutes the en-route reference per-call latency declared in
    # upfront_scenarios.yaml (llm_latency_ref_s, with provenance).
    from upfront_ablation import parse_trial
    row = parse_trial(_fixture("open_set_A2_qwen3_14b.txt"),
                      llm_latency_ref_s=1.3)
    assert abs(row["time_to_resolution_minus_llm_s"]
               - (row["time_to_resolution_s"] - 2.8)) < 0.01
    assert abs(row["time_to_resolution_norm_llm_s"]
               - (row["time_to_resolution_minus_llm_s"] + 2 * 1.3)) < 0.01
    # No reference supplied -> no normalized estimate (never silently invent).
    row = parse_trial(_fixture("open_set_A2_qwen3_14b.txt"))
    assert row["time_to_resolution_norm_llm_s"] == ""
    # Control run, no LLM recovery calls: both equal the measured resolution.
    row = parse_trial(FIXTURE_CONTROL, llm_latency_ref_s=1.3)
    assert row["time_to_resolution_minus_llm_s"] == row["time_to_resolution_s"]
    assert row["time_to_resolution_norm_llm_s"] == row["time_to_resolution_s"]


def test_scenarios_yaml_declares_the_latency_reference():
    cfg = yaml.safe_load(open(SCENARIOS_PATH))
    assert cfg["common"]["llm_latency_ref_s"] == 1.3


# Real U-B/U2 r5 (2026-08-02) shape: after the operator opened, every poll
# read plan_ok=True barrier_ok=False — the clearance gate, evaluated at a
# centroid the re-diagnosis had drifted ~2m off the partition, vetoed a
# dispatch the planner had approved, until the cap forced escalation. The
# gate must never be cited as CLEARANCE evidence (near-no-op), but as a
# failure MECHANISM it is load-bearing and the row must expose it.
FIXTURE_BARRIER_VETO = """\
[TRIAL] scenario=U-B arm=U2 rep=5 commit=deadbee start=1785681800
response:
semantic_nav_interfaces.srv.NavigateToQuery_Response(success=False, outcome='NEEDS_OPERATOR', failure_reason="Could not reach 'bed'.", reached_target='')
[navigation_orchestrator-25] [INFO] [1785681810.0] [navigation_orchestrator]: [UP_FRONT] Pre-flight validation failed; entering up-front blockage recovery.
[navigation_orchestrator-25] [INFO] [1785681947.7] [navigation_orchestrator]: [UP_FRONT] Operator decision for 'open_door_then_replan': acknowledged=True note='operator_confirmed'
[navigation_orchestrator-25] [INFO] [1785681961.8] [navigation_orchestrator]: [UP_FRONT] Recheck poll=0: barrier=still_blocked plan_ok=True barrier_ok=False
[navigation_orchestrator-25] [INFO] [1785681970.0] [navigation_orchestrator]: [UP_FRONT] Recheck poll=1: barrier=still_blocked plan_ok=True barrier_ok=False
"""


def test_barrier_gate_veto_is_exposed_as_a_failure_mechanism():
    from upfront_ablation import parse_trial
    row = parse_trial(FIXTURE_BARRIER_VETO)
    assert row["plan_ok_final"] is True
    assert row["barrier_gate_vetoed"] is True
    assert row["terminal_outcome"] == "needs-operator"
    # Success path (14B): the gate never vetoed a valid plan.
    row = parse_trial(_fixture("open_set_A2_qwen3_14b.txt"))
    assert row["barrier_gate_vetoed"] is False
    # Control: no polls at all.
    assert parse_trial(FIXTURE_CONTROL)["barrier_gate_vetoed"] == ""


def test_no_barrier_clear_column():
    # The up-front barrier-clear gate is a known near-no-op ('cleared' is
    # unreachable to falsify for a 1-cell barrier). Recording it invites
    # someone to cite it later — the plan forbids the column outright.
    from upfront_ablation import parse_trial
    row = parse_trial(FIXTURE_CONTROL)
    assert not [k for k in row if "barrier_clear" in k]


# ---------------------------------------------------------------- scenario
# yaml + runner script sanity.

def test_scenarios_yaml_complete():
    cfg = yaml.safe_load(open(SCENARIOS_PATH))
    assert cfg["common"]["nl_command"] == "tired"
    assert cfg["common"]["goal_object_key"] == "bed:120"
    assert len(cfg["common"]["blocker_xy"]) == 2

    arms = cfg["arms"]
    assert set(arms) == {"U0", "U1", "U2"}
    assert arms["U0"]["up_front_recovery_enabled"] is False
    assert arms["U1"]["up_front_recovery_enabled"] is True
    assert arms["U2"]["up_front_recovery_enabled"] is True
    assert arms["U1"]["open_set_inference_enabled"] is False
    assert arms["U2"]["open_set_inference_enabled"] is True
    # U0 must run the GEOMETRIC en-route tree: with up-front recovery off the
    # goal dispatches straight into the Nav2 BT, and the default (LLM) tree
    # would rescue the baseline it is supposed to isolate.
    assert "geometric" in arms["U0"]["recovery_bt_xml"]
    assert "geometric" not in arms["U1"]["recovery_bt_xml"]
    assert "geometric" not in arms["U2"]["recovery_bt_xml"]

    sc = cfg["scenarios"]
    assert set(sc) == {"U-A", "U-B"}
    assert sc["U-A"]["blocker_present"] is False
    assert sc["U-B"]["blocker_present"] is True
    assert sc["U-B"]["expected_final_directive"]["U2"] == (
        "open_door_then_replan")
    for name in ("U-A", "U-B"):
        for key in ("blocker_present", "expected_upfront_trigger",
                    "expected_final_directive"):
            assert key in sc[name], f"{name} missing {key}"


def test_runner_script_is_valid_bash_and_reads_the_yaml():
    assert os.path.exists(RUNNER_PATH)
    subprocess.run(["bash", "-n", RUNNER_PATH], check=True)
    src = open(RUNNER_PATH).read()
    assert "upfront_scenarios.yaml" in src
    assert "/parse_semantic_command" in src
    assert "/navigate_to_query" in src
    assert "map_residual_check.py" in src
    assert "/operator_decision" in src


def test_runner_requires_pre_observed_blocker_instead_of_spawning():
    # A panel spawned seconds before dispatch is in NEITHER /map nor the
    # costmap (the robot never observed it), so pre-flight validation would
    # succeed and the up-front lane would never trigger. The protocol is:
    # map open -> spawn -> OBSERVE (panel enters /map) -> run. The runner
    # must therefore never spawn the blocker itself, and must gate U-B on
    # the partition coordinate reading OCCUPIED.
    src = open(RUNNER_PATH).read()
    # Comments and operator-guidance echos may mention the spawn script; the
    # EXECUTABLE lines must not invoke it (or any spawner).
    executable = [ln for ln in src.splitlines()
                  if not ln.strip().startswith("#")
                  and not ln.strip().startswith("echo")]
    assert not [ln for ln in executable if "close_partition" in ln]
    assert not [ln for ln in executable if "spawn" in ln.lower()]
    # Both gate polarities exist: U-A demands clear, U-B demands occupied.
    assert 'BLOCKER_PRESENT" = "True"' in src
    assert "OCCUPIED" in src
