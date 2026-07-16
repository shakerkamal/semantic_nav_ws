"""Tests for the en-route ablation harness (eval/ scripts are not a package,
so tests add eval/ to sys.path)."""
import os
import sys

import yaml

EVAL_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, EVAL_DIR)

SCENARIOS_PATH = os.path.join(EVAL_DIR, "enroute_scenarios.yaml")

REQUIRED_SCENARIO_KEYS = {
    "goal_query", "nl_command", "intent_hint", "trigger",
    "blocker", "detector", "expected_directive",
}


import re
import xml.etree.ElementTree as ET

WS_ROOT = os.path.dirname(EVAL_DIR)
BT_CONFIG_DIR = os.path.join(
    WS_ROOT, "src", "semantic_nav_nav2_plugins", "config")
BLLM_BT = os.path.join(BT_CONFIG_DIR, "semantic_recovery_bt.xml")
GEO_BT = os.path.join(BT_CONFIG_DIR, "semantic_recovery_bt_geometric.xml")


def test_bt_xmls_are_well_formed():
    # colcon build never validates XML syntax (config/ is a directory
    # install), so a broken tree can sit undetected until Nav2 tries to load
    # it at runtime. Caught live 2026-07-15: a literal "--" inside an XML
    # comment's TEXT (not the <!-- / --> delimiters) is illegal per the XML
    # spec and silently passed every prior check in this file.
    for path in (BLLM_BT, GEO_BT):
        ET.parse(path)   # raises ET.ParseError if malformed


def _tier1_block(path):
    src = open(path).read()
    m = re.search(r"<PipelineSequence.*?</PipelineSequence>", src, re.S)
    assert m, f"no PipelineSequence found in {path}"
    return m.group(0)


def test_tier1_byte_parity_between_bllm_and_bgeo():
    # B-GEO must differ from B-LLM in the recovery child ONLY (plan invariant).
    # A persisted version of the one-off parity check from Task 2, so future
    # Tier-1 edits (like the PersistentBlockageGate below) can't silently land
    # in only one of the two trees.
    assert _tier1_block(BLLM_BT) == _tier1_block(GEO_BT)


def test_bllm_retreats_to_a_standoff_after_the_post_approach_query():
    # 2026-07-16 S2: when the pass-1 wide query finds no candidate (the robot
    # is still ~2.5m out when Tier-3 starts), the blind DriveOnHeading
    # collision-stops the robot ~0.4m from the blocker at an arbitrary angle
    # where the 59-degree camera sees only a sliver of it. Pass-2 then
    # perceives and matches the object -- but nothing acted on that match,
    # so the robot escalated (and later re-observed after the operator
    # cleared the blocker) from the jammed pose. Once pass-2 has a
    # candidate, the tree must retreat to a standoff computed from the
    # PERCEIVED bbox (ComputeStandoffPose's yaw faces the object) before
    # EscalateToLLMRecovery -- ForceSuccess-wrapped: failing to reach the
    # standoff must never abort the semantic branch itself.
    root = ET.parse(BLLM_BT).getroot()
    branch = root.iter("Sequence")
    branch = [s for s in branch if s.get("name") == "SemanticRecoveryBranch"]
    assert len(branch) == 1
    children = list(branch[0])

    query_idxs = [
        i for i, el in enumerate(children) if el.tag == "QuerySemanticContext"
    ]
    assert len(query_idxs) == 2, "expected the pass-1 and pass-2 queries"
    escalate_idxs = [
        i for i, el in enumerate(children) if el.tag == "EscalateToLLMRecovery"
    ]
    assert len(escalate_idxs) == 1

    retreat = [
        el for el in children[query_idxs[1] + 1:escalate_idxs[0]]
        if el.tag == "ForceSuccess"
        and el.find(".//ComputeStandoffPose") is not None
    ]
    assert retreat, (
        "no ForceSuccess-wrapped standoff retreat between the pass-2 query"
        " and EscalateToLLMRecovery"
    )
    gate = retreat[0].find(".//HasResponsibleObjectCandidate")
    assert gate is not None, "retreat must be gated on a pass-2 candidate"
    follow = retreat[0].find(".//FollowPath")
    assert follow is not None and follow.get("path") == "{standoff_path}"


def test_tier1_has_persistent_blockage_gate_in_both_trees():
    # 2026-07-15: SmacPlanner2D can keep finding a marginal detour around a
    # soft/partial obstruction every 1Hz replan cycle while the rotation shim
    # re-orients toward each new heading; PoseProgressChecker credits that
    # rotation as "progress" and the controller never hard-fails, so a
    # persistent-but-not-fully-sealed blockage can livelock Tier 1 forever
    # without ever escalating to Tier 2/3. PathClearCondition's existing
    # severity+debounce gate (already proven in SignalWaitRecheck) is reused
    # as the FIRST child of the Tier-1 PipelineSequence: PipelineSequence
    # re-ticks all prior children while a later one is RUNNING, so a FAILURE
    # here aborts the running FollowPath and bubbles to the outer
    # RecoveryNode(3) -- the missing complement to "planner keeps nominally
    # succeeding." Small/transient blockages still pass through untouched via
    # allow_geometric_detour_first.
    for path in (BLLM_BT, GEO_BT):
        block = _tier1_block(path)
        gate = re.search(r"<PathClearCondition\b.*?/>", block, re.S)
        assert gate, f"PersistentBlockageGate missing from Tier-1 in {path}"
        no_comments = re.sub(r"<!--.*?-->", "", block, flags=re.S)
        first_child = re.search(r"<PipelineSequence[^>]*>\s*<(\w+)", no_comments)
        assert first_child.group(1) == "PathClearCondition", (
            f"PathClearCondition must be the FIRST child of the Tier-1 "
            f"PipelineSequence in {path} so it is reticked every cycle")


def _semantic_recovery_branch_no_comments():
    src = open(BLLM_BT).read()
    no_comments = re.sub(r"<!--.*?-->", "", src, flags=re.S)
    m = re.search(
        r"<Sequence name=\"SemanticRecoveryBranch\">(.*?)</Sequence>\s*"
        r"</RoundRobin>",
        no_comments, re.S)
    assert m, "SemanticRecoveryBranch not found in semantic_recovery_bt.xml"
    return m.group(1)


def test_tier3_queries_wide_before_deciding_how_to_approach():
    # Part A (2026-07-15): a blind DriveOnHeading shove to the collision
    # boundary (the ORIGINAL fix for the stopping-distance problem, see
    # git history) still only ever produced 'inferred' matches, never
    # 'verified' -- proximity alone doesn't fix attribution quality.
    # Redesigned: query WIDE from wherever the robot already is (no approach
    # yet) to discover any known candidate; if one exists, compute a REAL
    # standoff from its bbox and navigate there properly; only fall back to
    # the blind approach if nothing is known. Sequence must be:
    #   CaptureBlockageContext -> QuerySemanticContext -> Fallback(
    #     Sequence(HasResponsibleObjectCandidate, ComputeStandoffPose,
    #              ComputePathToPose/FollowPath to the standoff),
    #     ForceSuccess(DriveOnHeading))
    #   -> CaptureBlockageContext -> QuerySemanticContext (pass 2, re-sample
    #      now that the robot is close) -> EscalateToLLMRecovery.
    branch = _semantic_recovery_branch_no_comments()

    capture_positions = [m.start() for m in re.finditer(r"<CaptureBlockageContext\b", branch)]
    query_positions = [m.start() for m in re.finditer(r"<QuerySemanticContext\b", branch)]
    fallback_pos = branch.find("<Fallback name=\"ApproachBlockage\">")
    has_candidate_pos = branch.find("<HasResponsibleObjectCandidate")
    standoff_pos = branch.find("<ComputeStandoffPose")
    drive_on_heading_pos = branch.find("<DriveOnHeading")
    escalate_pos = branch.find("<EscalateToLLMRecovery")

    assert len(capture_positions) == 2, "expected 2 CaptureBlockageContext passes"
    assert len(query_positions) == 2, "expected 2 QuerySemanticContext passes"
    for pos in (fallback_pos, has_candidate_pos, standoff_pos,
                drive_on_heading_pos, escalate_pos):
        assert pos != -1

    # Strict ordering: pass 1 (capture, query) -> Fallback(standoff-approach
    # containing the gate+compute, blind-approach containing DriveOnHeading)
    # -> pass 2 (capture, query) -> escalate.
    assert capture_positions[0] < query_positions[0] < fallback_pos
    assert fallback_pos < has_candidate_pos < standoff_pos < drive_on_heading_pos
    assert drive_on_heading_pos < capture_positions[1] < query_positions[1] < escalate_pos

    # DriveOnHeading is still ForceSuccess-wrapped (returns FAILED on both a
    # detected collision ahead and on timeout; only reaching the full
    # requested distance returns SUCCESS -- the branch must continue
    # regardless of which outcome stopped it).
    m = re.search(
        r"<ForceSuccess>\s*<DriveOnHeading\b([^/]*)/>\s*</ForceSuccess>",
        branch, re.S)
    assert m, "DriveOnHeading must stay ForceSuccess-wrapped"
    assert 'server_name="drive_on_heading"' in m.group(1)


def test_query_semantic_context_passes_carry_bbox_ports():
    # Both QuerySemanticContext passes must surface
    # responsible_bbox_center/extent -- pass 1 feeds ComputeStandoffPose;
    # pass 2 feeds EscalateToLLMRecovery/logging. (ComputeStandoffPose ALSO
    # binds the same attribute string as its own input port, so a blind
    # whole-branch substring count isn't precise enough here.)
    branch = _semantic_recovery_branch_no_comments()
    query_elements = re.findall(r"<QuerySemanticContext\b[^>]*/>", branch)
    assert len(query_elements) == 2
    for element in query_elements:
        assert 'responsible_bbox_center="{responsible_bbox_center}"' in element
        assert 'responsible_bbox_extent="{responsible_bbox_extent}"' in element


def test_geometric_bt_has_no_semantic_branch():
    tree = ET.parse(GEO_BT)
    tags = {el.tag for el in tree.iter()}
    # The whole Tier-3 branch must be gone.
    for forbidden in ("CaptureBlockageContext", "QuerySemanticContext",
                      "EscalateToLLMRecovery", "OperatorPrompt", "Switch3"):
        assert forbidden not in tags, f"{forbidden} must not be in B-GEO"
    # Tier 1 + Tier 2 must survive intact.
    for required in ("ValidateSemantic", "RateController", "ComputePathToPose",
                     "FollowPath", "ClearEntireCostmap", "BackUp"):
        assert required in tags, f"{required} missing from B-GEO"
    # Tier-1 PARITY with the current semantic tree (158c726): the outer
    # RecoveryNode retries 3x, and BOTH in-place single-retry wraps survive —
    # B-GEO must differ from B-LLM in the recovery child ONLY.
    recoveries = {el.get("name"): el.get("number_of_retries")
                  for el in tree.iter("RecoveryNode")}
    assert recoveries.get("SemanticRecovery") == "3"
    assert recoveries.get("ComputePathToPose") == "1"
    assert recoveries.get("FollowPath") == "1"
    assert len(recoveries) == 3


def test_scenarios_yaml_complete():
    with open(SCENARIOS_PATH) as f:
        data = yaml.safe_load(f)
    assert set(data["scenarios"].keys()) == {"S1", "S2", "S3", "S4", "S5"}
    for name, sc in data["scenarios"].items():
        missing = REQUIRED_SCENARIO_KEYS - set(sc.keys())
        assert not missing, f"{name} missing {missing}"
    # GT directives the parser scores against (spec section 3).
    assert data["scenarios"]["S1"]["expected_directive"] == "none"
    assert data["scenarios"]["S2"]["expected_directive"] == "open_door_then_replan"
    assert data["scenarios"]["S3"]["expected_directive"] == "clear_object_then_replan"
    assert data["scenarios"]["S4"]["expected_directive"] == "wait_then_replan"
    assert data["scenarios"]["S5"]["expected_directive"] == "retry_target"
    # S2's door:119 is a persistent-map object, but attribution via a wide
    # static-map lookup alone proved unreliable (2026-07-15: it only ever
    # produced 'inferred', never 'verified', matches). Mirrors S3/S4: the
    # detector reports what it actually perceives, so match_responsible_object
    # can use the dynamic-preferred tie-break (commit a6f5e9c) here too.
    assert data["scenarios"]["S2"]["detector"] is not None
    assert data["scenarios"]["S2"]["detector"]["tag"] == "door"
    # Perception-only contract: detectors never carry affordance fields.
    for name, sc in data["scenarios"].items():
        det = sc["detector"]
        if det is not None:
            assert "openable" not in det and "clearable" not in det \
                and "safety_class" not in det, \
                f"{name}: detector must publish perception only"


def test_planar_dist():
    from enroute_common import planar_dist
    assert abs(planar_dist((0.0, 0.0), (3.0, 4.0)) - 5.0) < 1e-9


def test_load_scenarios_merges_common():
    from enroute_common import load_scenarios
    data = load_scenarios(SCENARIOS_PATH)
    s4 = data["scenarios"]["S4"]
    # common block is exposed alongside scenarios
    assert data["common"]["perception_range_m"] == 3.0
    assert s4["detector"]["tag"] == "person"
    assert s4["delete_after_sec"] == 40.0


def test_trigger_line_crossing():
    from enroute_blockage_trigger import crossed
    # S1/S2 style: robot driving east, fires once x exceeds 2.0
    assert not crossed("x", 2.0, "increasing", (1.9, 0.0))
    assert crossed("x", 2.0, "increasing", (2.1, 0.0))
    # S3/S4/S5 style: robot driving west, fires once x drops below -1.0
    assert not crossed("x", -1.0, "decreasing", (-0.5, 0.0))
    assert crossed("x", -1.0, "decreasing", (-1.2, 0.0))
    assert crossed("y", 1.0, "increasing", (0.0, 1.5))


def test_database_include_xml_matches_gazebo_ros_template():
    from enroute_blockage_trigger import database_include_xml
    xml = database_include_xml(
        "aws_robomaker_residential_Door_01", "scenario_door")
    # Mirrors gazebo_ros's OWN spawn_entity.py MODEL_DATABASE_TEMPLATE
    # (<world><include>, NO pose inside the xml -- placement comes entirely
    # from the SpawnEntity service's separate initial_pose field), PLUS an
    # explicit SDF <include><name> override. gazebo_ros_factory only applies
    # request.name to a direct <model>/<light> element, never to an
    # <include>, so without the override the inserted model keeps (or, on a
    # leaf-name collision with a model nested inside the world's own
    # <model><include> wrappers, gets auto-renamed FROM) its model.sdf name
    # -- and DeleteEntity by the request name then fails with 'does not
    # exist' while the blocker stays in the world (S2, 2026-07-16).
    assert "<world" in xml
    assert "<include>" in xml
    assert "<name>scenario_door</name>" in xml
    assert "<uri>model://aws_robomaker_residential_Door_01</uri>" in xml
    assert "<pose>" not in xml
    assert "<model " not in xml and "<model>" not in xml


def test_database_blocker_entities_do_not_shadow_their_model_name():
    # An entity named exactly like its database model collides with the
    # same-named model NESTED inside the world's own include wrappers
    # (e.g. small_house*.world's <model name='Door_01_001'><include>
    # aws_robomaker_residential_Door_01), which is what made gzserver
    # auto-rename the spawned S2 door so DeleteEntity could never find it.
    with open(SCENARIOS_PATH) as f:
        scenarios = yaml.safe_load(f)["scenarios"]
    for name, scenario in scenarios.items():
        blocker = scenario.get("blocker", {})
        if blocker.get("kind") == "database":
            assert blocker["entity"] != blocker["model"], (
                f"{name}: blocker entity must not equal the database model name"
            )


FIXTURE_TRIAL = """\
[TRIAL] scenario=S5 variant=bllm_retry rep=1 commit=abc1234 start=1783880000
[navigation_orchestrator-25] [INFO] [1783880010.100000000] [navigation_orchestrator]: [EXECUTION] Sending goal to execute_pose action server (object_key='bed:120', db_version=1193208084, db_stamp=1.0): frame='map', x=-4.8, y=2.2
[behavior_server-19] [INFO] [1783880020.000000000] [behavior_server]: Running backup
[navigator_node-10] [WARN] [1783880025.000000000] [navigator_node]: [RECOVERY] LLM recovery invoked. original_target='bed:120', failure_stage='execution', trigger_source='bt_recovery_plugin', match_type='unknown', responsible_object_key='', nav2_message='path blocked or navigation aborted', remaining_retry_budget=3
[navigation_orchestrator-25] [INFO] [1783880027.500000000] [navigation_orchestrator]: [RECOVERY/BT] BT proposal response: success=True, action='retry_target', target_object_tag='couch', target_intent_hint='a place to rest', confidence=80, message='ok'
[navigation_orchestrator-25] [INFO] [1783880027.600000000] [navigation_orchestrator]: [RECOVERY/BT] eligible=['retry_target', 'give_up'] llm='retry_target' -> action=retry_target (overridden=False reason=llm_selected)
[navigation_orchestrator-25] [INFO] [1783880027.700000000] [navigation_orchestrator]: [RECOVERY/BT] Retry target redirected from blocked 'bed:120' to reachable alternative 'couch:33' (tag='couch').
[navigation_orchestrator-25] [INFO] [1783880060.000000000] [navigation_orchestrator]: [EXECUTION] Executor finished with status=SUCCEEDED(4), success=True, object_key='bed:120', db_version=1193208084, db_stamp=1.0, message='Navigation succeeded'
[TRIAL] end=1783880061
[MOCK_DETECTOR] dist=2.80 publishing=True
[MOCK_DETECTOR] dist=1.10 publishing=True
[MOCK_DETECTOR] dist=1.90 publishing=True
[MOCK_DETECTOR] dist=0.95 publishing=True
"""


def test_parse_trial_s5_redirected_run():
    from enroute_ablation import parse_trial
    row = parse_trial(FIXTURE_TRIAL, expected_directive="retry_target")
    assert row["scenario"] == "S5"
    assert row["variant"] == "bllm_retry"
    assert row["rep"] == 1
    assert row["terminal_outcome"] == "intent-preserving-alternative"
    assert row["resolving_tier"] == "T3"
    assert row["directive_chosen"] == "retry_target"
    assert row["directive_correct"] is True
    assert row["target_object_tag"] == "couch"
    assert row["recovery_cycles"] == 1
    assert row["llm_calls"] == 1
    assert abs(row["llm_latency_s"] - 2.5) < 0.01
    assert abs(row["time_to_resolution_s"] - 49.9) < 0.2
    assert row["min_standoff_m"] == 0.95
    assert row["reapproach_count"] == 2   # two descents below 1.5 after being above
    assert row["db_version"] == "1193208084"
    assert row["code_commit"] == "abc1234"


def test_parse_trial_geo_abort():
    text = FIXTURE_TRIAL.replace("variant=bllm_retry", "variant=bgeo")
    # strip every T3 and redirect line, flip the terminal to failure
    lines = [l for l in text.splitlines()
             if "RECOVERY/BT" not in l and "LLM recovery invoked" not in l]
    lines = [l.replace("status=SUCCEEDED(4), success=True",
                       "status=ABORTED(6), success=False") for l in lines]
    from enroute_ablation import parse_trial
    row = parse_trial("\n".join(lines), expected_directive="retry_target")
    assert row["terminal_outcome"] == "aborted"
    assert row["resolving_tier"] == "T2"
    assert row["llm_calls"] == 0
    assert row["directive_chosen"] == "none"


# A recovery-exhausted run ends with the NavigateToQuery service returning
# NEEDS_OPERATOR — there is NO "Executor finished" line. The terminal outcome,
# db_version, and time_to_resolution must come from the response + dispatch.
FIXTURE_NEEDS_OPERATOR = """\
[TRIAL] scenario=S1 variant=bllm rep=1 commit=625d2e2 start=1784111956
requester: making request: semantic_nav_interfaces.srv.NavigateToQuery_Request(query='refrigerator:6', nl_command='', intent_hint='')

response:
semantic_nav_interfaces.srv.NavigateToQuery_Response(success=False, outcome='NEEDS_OPERATOR', failure_reason="Could not reach 'refrigerator:6'. Geometric and semantic recovery were exhausted and no reachable alternative was found. Operator input required.", reached_target='')

[TRIAL] end=1784111978
[navigation_orchestrator-25] [INFO] [1784111956.982362773] [navigation_orchestrator]: [EXECUTION] Sending goal to execute_pose action server (object_key='refrigerator:6', db_version=3498918824, db_stamp=1784033173.75): frame='map', x=7.117, y=-0.780
[behavior_server-13] [INFO] [1784111965.400297694] [behavior_server]: Running backup
[navigator_node-24] [WARN] [1784111970.396543454] [navigator_node]: [RECOVERY] LLM recovery invoked. original_target='refrigerator:6', failure_stage='execution', trigger_source='bt_recovery_plugin', match_type='unknown', responsible_object_key='', nav2_message='path blocked or navigation aborted', remaining_retry_budget=3
[navigation_orchestrator-25] [INFO] [1784111971.232631218] [navigation_orchestrator]: [RECOVERY/BT] BT proposal response: success=True, action='give_up', target_object_tag='', target_intent_hint='', confidence=100, message='LLM recovery chose give_up.'
[behavior_server-13] [INFO] [1784111978.024669882] [behavior_server]: backup completed successfully
"""


# A silent successful run: after validation the semantic nodes go quiet for the
# whole drive, so the buffered "Executor finished" line is lost from the slice.
# The wrapper's wall-clock markers must supply time_to_resolution instead.
FIXTURE_WALL_TIMED = """\
[TRIAL] scenario=S1 variant=bllm rep=1 commit=76cd818 start=1784121414
[TRIAL] dispatch_wall=1784121415.500000000
response:
semantic_nav_interfaces.srv.NavigateToQuery_Response(success=True, outcome='REACHED', failure_reason='', reached_target='refrigerator:6')
[TRIAL] finish_wall=1784121433.600000000
[TRIAL] end=1784121433
[navigation_orchestrator-25] [INFO] [1784121415.589255104] [navigation_orchestrator]: [EXECUTION] Sending goal to execute_pose action server (object_key='refrigerator:6', db_version=3498918824, db_stamp=1784033173.75): frame='map', x=7.121, y=-0.736
[INFO] [1784121420.607038790] [enroute_blockage_trigger]: [TRIGGER] spawned 'scenario_bucket' at (3.621, -0.542)
"""


def test_parse_trial_wall_clock_fallback():
    from enroute_ablation import parse_trial
    row = parse_trial(FIXTURE_WALL_TIMED, expected_directive="none")
    assert row["terminal_outcome"] == "original-target-reached"
    assert row["resolving_tier"] == "none"
    assert row["directive_chosen"] == "none"
    # No Executor-finished line (lost to the buffer race): resolution must come
    # from the wall markers (18.1 s), NOT the last stamp (the +5 s spawn line).
    assert abs(row["time_to_resolution_s"] - 18.1) < 0.05


def test_parse_trial_needs_operator():
    from enroute_ablation import parse_trial
    row = parse_trial(FIXTURE_NEEDS_OPERATOR, expected_directive="none")
    assert row["terminal_outcome"] == "needs-operator"
    assert row["resolving_tier"] == "T3"
    assert row["directive_chosen"] == "give_up"
    assert row["llm_calls"] == 1
    # db_version and time_to_resolution come from the dispatch line + last stamp,
    # since there is no Executor-finished line to read them from.
    assert row["db_version"] == "3498918824"
    assert abs(row["time_to_resolution_s"] - 21.042) < 0.1
    assert row["code_commit"] == "625d2e2"
