#!/usr/bin/env bash
# Usage: bash eval/run_enroute_trial.sh <S1|S2|S3|S4|S5> <bgeo|bllm|bllm_retry> <rep>
#
# One en-route ablation trial on the UGV ROVER. PRECONDITIONS (checklist):
#   - stack up via aws_small_house_ugv_semantic.launch.py (rover env:
#     source ~/ugv_sim_env.bash), piped through eval/log_session.sh with
#     LOG_KEY_PATTERN extended to include behavior_server|bt_navigator
#     (this script slices the newest *key.log);
#     B-GEO runs relaunch with recovery_bt_xml:=$(ros2 pkg prefix
#     semantic_nav_nav2_plugins)/share/semantic_nav_nav2_plugins/config/semantic_recovery_bt_geometric.xml
#   - robot mapped and back at the start pose; blocker corridor OPEN —
#     REAPPEARING-OBSTACLE CHECK between reps: after a blocker was deleted,
#     confirm the corridor reads free in /map before the next rep (RTAB-Map
#     can re-assert old per-node grids on graph optimization; manual remedy:
#     ros2 service call /rtabmap/cleanup_local_grids WHILE the map shows the
#     cell empty — deliberately not automated here)
#   - navigation_terminal running in its own TTY (S2/S3 operator prompts)
#   - one warmup LLM call done after any relaunch
#   - exactly one navigator_node alive
set -euo pipefail
SCEN=$1; VARIANT=$2; REP=$3
EVAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$EVAL_DIR/logs/enroute_${SCEN}_${VARIANT}_r${REP}.log"

# HARD pre-flight gate: a stale teleop publishing zero twists races the
# velocity smoother (gazebo diff_drive has no cmd_vel timeout) and fakes a
# controller bug — it poisons the trial SILENTLY, so refuse to run. Expected
# publishers: velocity_smoother + behavior_server, nothing else.
CMDVEL_PUBS=$(ros2 topic info /cmd_vel -v | awk '/Publisher count:/{print $3}')
if [ "$CMDVEL_PUBS" -ne 2 ]; then
  echo "ABORT: /cmd_vel has $CMDVEL_PUBS publishers (expected 2:"
  echo "velocity_smoother + behavior_server). Offending nodes:"
  ros2 topic info /cmd_vel -v
  echo "Kill the stale publisher (usually teleop_twist_keyboard) and retry."
  exit 1
fi

RUN_LOG=$(ls -t "$EVAL_DIR"/logs/*key.log | head -1)
[ -n "$RUN_LOG" ] || { echo "no *key.log found — is the stack logging?"; exit 1; }
START_LINE=$(wc -l < "$RUN_LOG")

# S5 arm switch (live param, no relaunch): bllm_retry enables retry_target.
if [ "$VARIANT" = "bllm_retry" ]; then
  ros2 param set /navigation_orchestrator enroute_retry_target_enabled true
else
  ros2 param set /navigation_orchestrator enroute_retry_target_enabled false
fi

# Scenario constants for the goal call.
QUERY=$(python3 -c "
import yaml; sc=yaml.safe_load(open('$EVAL_DIR/enroute_scenarios.yaml'))['scenarios']['$SCEN']
print(sc['goal_query'])")
NLCMD=$(python3 -c "
import yaml; sc=yaml.safe_load(open('$EVAL_DIR/enroute_scenarios.yaml'))['scenarios']['$SCEN']
print(sc['nl_command'])")
HINT=$(python3 -c "
import yaml; sc=yaml.safe_load(open('$EVAL_DIR/enroute_scenarios.yaml'))['scenarios']['$SCEN']
print(sc['intent_hint'])")

COMMIT=$(git -C "$EVAL_DIR/.." rev-parse --short HEAD)
CHILD_LOG=$(mktemp)

# World-changer and (S3/S4 only) perception stand-in, in the background.
python3 "$EVAL_DIR/enroute_blockage_trigger.py" --scenario "$SCEN" \
  >> "$CHILD_LOG" 2>&1 &
TRIG_PID=$!
DET_PID=""
if [ "$SCEN" = "S3" ] || [ "$SCEN" = "S4" ]; then
  python3 "$EVAL_DIR/mock_concept_graph_detector.py" --scenario "$SCEN" \
    >> "$CHILD_LOG" 2>&1 &
  DET_PID=$!
fi

echo "[TRIAL] scenario=$SCEN variant=$VARIANT rep=$REP commit=$COMMIT start=$(date +%s)" | tee "$OUT"

# Blocks until the pipeline finishes (success, abort, or operator handoff).
ros2 service call /navigate_to_query semantic_nav_interfaces/srv/NavigateToQuery \
  "{query: '$QUERY', nl_command: '$NLCMD', intent_hint: '$HINT'}" \
  | tee -a "$OUT"

echo "[TRIAL] end=$(date +%s)" >> "$OUT"

kill $TRIG_PID 2>/dev/null || true
[ -n "$DET_PID" ] && kill $DET_PID 2>/dev/null || true

# Slice the session key-log to this trial's window and append child output.
tail -n +"$((START_LINE + 1))" "$RUN_LOG" >> "$OUT"
cat "$CHILD_LOG" >> "$OUT"
rm -f "$CHILD_LOG"

echo "[TRIAL] wrote $OUT"
