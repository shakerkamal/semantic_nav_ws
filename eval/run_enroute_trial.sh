#!/usr/bin/env bash

# Usage: bash eval/run_enroute_trial.sh <S1|S2|S3|S4|S5> <bgeo|bllm|bllm_retry> <rep>
#
# One en-route ablation trial on the UGV ROVER. PRECONDITIONS (checklist):
# - stack up via aws_small_house_ugv_semantic.launch.py (rover env:
#   source ~/ugv_sim_env.bash), piped through eval/log_session.sh with
#   LOG_KEY_PATTERN extended to include behavior_server|bt_navigator
#   (this script slices the newest *key.log);
#   B-GEO runs relaunch with recovery_bt_xml:=$(ros2 pkg prefix
#   semantic_nav_nav2_plugins)/share/semantic_nav_nav2_plugins/config/semantic_recovery_bt_geometric.xml
# - robot mapped and back at the start pose; blocker corridor OPEN —
#   REAPPEARING-OBSTACLE CHECK between reps: after a blocker was deleted,
#   confirm the corridor reads free in /map before the next rep (RTAB-Map
#   can re-assert old per-node grids on graph optimization; the M3 BT gate
#   now calls cleanup_local_grids only after /map and the global costmap
#   both confirm that the former blocker footprint is clear)
# - navigation_terminal running in its own TTY (S2/S3 operator prompts)
# - one warmup LLM call done after any relaunch
# - exactly one navigator_node alive

set -euo pipefail

SCEN=$1
VARIANT=$2
REP=$3

EVAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$EVAL_DIR/logs/enroute_${SCEN}_${VARIANT}_r${REP}.log"

# HARD pre-flight gate: a stale teleop publishing zero twists races the
# velocity smoother (gazebo diff_drive has no cmd_vel timeout) and fakes a
# controller bug — it poisons the trial SILENTLY, so refuse to run. Gate on
# the publisher NODE-NAME SET, not the count: Nav2's behavior_server creates
# one /cmd_vel publisher per behavior plugin (spin/backup/drive_on_heading/
# wait/assisted_teleop), so the healthy set is {velocity_smoother,
# behavior_server} at any multiplicity (6 on this rover). Anything else — a
# teleop_twist_keyboard, a second driver — is the poison we must catch.
STRAY=$(ros2 topic info /cmd_vel -v \
  | awk '/Node name:/{n=$3} /Endpoint type: PUBLISHER/{print n}' \
  | grep -vE '^(velocity_smoother|behavior_server)$' | sort -u || true)

if [ -n "$STRAY" ]; then
  echo "ABORT: unexpected /cmd_vel publisher(s):"
  echo "$STRAY"
  echo "Expected only velocity_smoother + behavior_server. Kill the stray"
  echo "publisher (usually teleop_twist_keyboard) and retry."
  exit 1
fi

# Operator-confirm scenarios (S2 door "opens", S3 chair "clears") end with an
# OperatorPrompt -> /operator_decision -> operator confirms -> the trigger
# deletes the spawned blocker. Without a live server for that service the
# prompt is undeliverable ("/operator_decision not ready before timeout"),
# the trial can only abort, and the blocker survives -- which looks EXACTLY
# like a deletion bug (burned a full S2 run on 2026-07-16 after a relaunch
# without restarting navigation_terminal). Refuse to start instead.
if [ "$SCEN" = "S2" ] || [ "$SCEN" = "S3" ]; then
  if ! ros2 service list 2>/dev/null | grep -qx "/operator_decision"; then
    echo "ABORT: no server for /operator_decision (navigation_terminal not running?)."
    echo "$SCEN needs an operator confirm to remove its blocker. Start the terminal"
    echo "in its own TTY, then retry:"
    echo "  ros2 run semantic_nav_orchestrator navigation_terminal --ros-args -p use_sim_time:=true"
    exit 1
  fi
fi

RUN_LOG=$(ls -t "$EVAL_DIR"/logs/*key.log | head -1)
[ -n "$RUN_LOG" ] || {
  echo "no *key.log found — is the stack logging?"
  exit 1
}
START_LINE=$(wc -l < "$RUN_LOG")

# S5 arm switch (live param, no relaunch): bllm_retry enables retry_target.
if [ "$VARIANT" = "bllm_retry" ]; then
  ros2 param set /navigation_orchestrator enroute_retry_target_enabled true
else
  ros2 param set /navigation_orchestrator enroute_retry_target_enabled false
fi

# Scenario constants for the goal call.
QUERY=$(python3 -c "
import yaml
sc = yaml.safe_load(open('$EVAL_DIR/enroute_scenarios.yaml'))['scenarios']['$SCEN']
print(sc['goal_query'])")

NLCMD=$(python3 -c "
import yaml
sc = yaml.safe_load(open('$EVAL_DIR/enroute_scenarios.yaml'))['scenarios']['$SCEN']
print(sc['nl_command'])")

HINT=$(python3 -c "
import yaml
sc = yaml.safe_load(open('$EVAL_DIR/enroute_scenarios.yaml'))['scenarios']['$SCEN']
print(sc['intent_hint'])")

# A trial run on uncommitted code must never masquerade as a clean revision:
# the header is the only link from a results row back to the implementation.
# Untracked files (logs, plots, csvs) do not affect the built code, so only
# tracked modifications mark the tree dirty. head/dirty_files/diff_sha256 pin
# the exact working-tree state a dirty run was produced from.
COMMIT=$(git -C "$EVAL_DIR/.." describe --always --dirty)
HEAD_COMMIT=$(git -C "$EVAL_DIR/.." rev-parse HEAD)
DIRTY_FILES=$(git -C "$EVAL_DIR/.." status --porcelain --untracked-files=no | wc -l)
DIFF_SHA256=$(git -C "$EVAL_DIR/.." diff --binary HEAD | sha256sum | awk '{print $1}')
CHILD_LOG=$(mktemp)

# World-changer and (S2/S3/S4 only) perception stand-in, in the background.
python3 "$EVAL_DIR/enroute_blockage_trigger.py" --scenario "$SCEN" \
  >> "$CHILD_LOG" 2>&1 &
TRIG_PID=$!

DET_PID=""
if [ "$SCEN" = "S2" ] || [ "$SCEN" = "S3" ] || [ "$SCEN" = "S4" ]; then
  python3 "$EVAL_DIR/mock_concept_graph_detector.py" --scenario "$SCEN" \
    >> "$CHILD_LOG" 2>&1 &
  DET_PID=$!
fi

echo "[TRIAL] scenario=$SCEN variant=$VARIANT rep=$REP commit=$COMMIT head=$HEAD_COMMIT dirty_files=$DIRTY_FILES diff_sha256=$DIFF_SHA256 start=$(date +%s)" \
  | tee "$OUT"

# Wall-clock markers bracket the service call so time_to_resolution survives the
# log-slice buffer race: on a silent successful drive the semantic nodes go
# quiet and the final Executor-finished line is still buffered when we slice.
echo "[TRIAL] dispatch_wall=$(date +%s.%N)" >> "$OUT"

# Blocks until the pipeline finishes (success, abort, or operator handoff).
ros2 service call /navigate_to_query semantic_nav_interfaces/srv/NavigateToQuery \
  "{query: '$QUERY', nl_command: '$NLCMD', intent_hint: '$HINT'}" \
  | tee -a "$OUT"

echo "[TRIAL] finish_wall=$(date +%s.%N)" >> "$OUT"
echo "[TRIAL] end=$(date +%s)" >> "$OUT"

kill "$TRIG_PID" 2>/dev/null || true
[ -n "$DET_PID" ] && kill "$DET_PID" 2>/dev/null || true

# Slice the session key-log to this trial's window and append child output.
tail -n +"$((START_LINE + 1))" "$RUN_LOG" >> "$OUT"
cat "$CHILD_LOG" >> "$OUT"
rm -f "$CHILD_LOG"

echo "[TRIAL] wrote $OUT"
