#!/usr/bin/env bash

# Usage: bash eval/run_upfront_trial.sh <U-A|U-B> <U0|U1|U2> <rep>
#
# One up-front ablation trial on the UGV ROVER (eval/upfront_evaluation_plan.md).
# PRECONDITIONS (checklist):
# - stack up via aws_small_house_ugv_semantic.launch.py (rover env:
#   source ~/ugv_sim_env.bash), piped through eval/log_session.sh
#   (this script slices the newest *key.log);
#   U0 relaunches with up_front_recovery_enabled:=false AND recovery_bt_xml:=
#   $(ros2 pkg prefix semantic_nav_nav2_plugins)/share/semantic_nav_nav2_plugins/config/semantic_recovery_bt_geometric.xml
#   (with up-front recovery off the goal dispatches straight into the Nav2 BT,
#   and the default LLM tree would rescue the baseline U0 exists to isolate)
# - robot mapped (fresh SLAM session per rep, corridor mapped OPEN incl. the
#   bedroom) and back near the start pose
# - U-B ONLY: panel inserted (close_partition.sh) AFTER mapping and then
#   OBSERVED so it is present in /map — the gate below verifies this; the
#   operator removes it (open_partition.sh) before confirming the prompt
# - navigation_terminal running in its own TTY (U-B operator prompt)
# - one warm-up LLM call done after any relaunch
# - exactly one navigator_node alive

set -euo pipefail

SCEN=$1
ARM=$2
REP=$3

EVAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$EVAL_DIR/logs/upfront_${SCEN}_${ARM}_r${REP}.log"

yamlget() {
  python3 -c "
import yaml
cfg = yaml.safe_load(open('$EVAL_DIR/upfront_scenarios.yaml'))
print($1)"
}

NLCMD=$(yamlget "cfg['common']['nl_command']")
BLOCKER_X=$(yamlget "cfg['common']['blocker_xy'][0]")
BLOCKER_Y=$(yamlget "cfg['common']['blocker_xy'][1]")
BLOCKER_PRESENT=$(yamlget "cfg['scenarios']['$SCEN']['blocker_present']")
ARM_UPFRONT=$(yamlget "cfg['arms']['$ARM']['up_front_recovery_enabled']")
ARM_OPENSET=$(yamlget "cfg['arms']['$ARM']['open_set_inference_enabled']")
ARM_BT=$(yamlget "cfg['arms']['$ARM']['recovery_bt_xml']")

# HARD pre-flight gate: a stale teleop publishing zero twists races the
# velocity smoother (gazebo diff_drive has no cmd_vel timeout) and fakes a
# controller bug — it poisons the trial SILENTLY, so refuse to run. Gate on
# the publisher NODE-NAME SET, not the count (behavior_server owns several).
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

# U-B ends (U2) or may end (U1/U0 dry-run guard) with an operator prompt via
# /operator_decision; without a live server the prompt is undeliverable and the
# run can only time out — which looks EXACTLY like a recovery bug.
if [ "$SCEN" = "U-B" ]; then
  if ! ros2 service list 2>/dev/null | grep -qx "/operator_decision"; then
    echo "ABORT: no server for /operator_decision (navigation_terminal not running?)."
    echo "Start it in its own TTY, then retry:"
    echo "  ros2 run semantic_nav_orchestrator navigation_terminal --ros-args -p use_sim_time:=true"
    exit 1
  fi
fi

# ARM ENFORCEMENT. up_front_recovery_enabled is CACHED at orchestrator init,
# so it can only be selected at launch — verify, never set. The behavior_tree
# param likewise pins which en-route BT a U0 dispatch would fall into.
LIVE_UPFRONT=$(ros2 param get /navigation_orchestrator up_front_recovery_enabled \
  | awk '{print tolower($NF)}')
WANT_UPFRONT=$(echo "$ARM_UPFRONT" | tr '[:upper:]' '[:lower:]')
if [ "$LIVE_UPFRONT" != "$WANT_UPFRONT" ]; then
  echo "ABORT: arm $ARM needs up_front_recovery_enabled=$WANT_UPFRONT but the"
  echo "live orchestrator has $LIVE_UPFRONT. This param is cached at node init:"
  echo "RELAUNCH with up_front_recovery_enabled:=$WANT_UPFRONT (it cannot be set live)."
  exit 1
fi

LIVE_BT=$(ros2 param get /navigation_orchestrator behavior_tree | awk '{print $NF}')
case "$ARM_BT" in
  *geometric*)
    if ! echo "$LIVE_BT" | grep -q "geometric"; then
      echo "ABORT: arm $ARM needs the GEOMETRIC en-route tree; relaunch with"
      echo "  recovery_bt_xml:=\$(ros2 pkg prefix semantic_nav_nav2_plugins)/share/semantic_nav_nav2_plugins/config/semantic_recovery_bt_geometric.xml"
      exit 1
    fi ;;
  *)
    if echo "$LIVE_BT" | grep -q "geometric"; then
      echo "ABORT: arm $ARM must run the default (LLM) en-route tree, but the"
      echo "live orchestrator has the geometric one. Relaunch without the override."
      exit 1
    fi ;;
esac

# open_set_inference_enabled IS read live at use-time — set it per arm.
ros2 param set /navigation_orchestrator open_set_inference_enabled "$ARM_OPENSET" >/dev/null
# U1 and U2 both keep LLM strategy selection on; only the affordance source
# differs (that is what makes the comparison interpretable).
if [ "$ARM" != "U0" ]; then
  ros2 param set /navigation_orchestrator up_front_llm_enabled true >/dev/null
fi

# Blocker-state gate on /map, polarity per scenario. The pre-flight
# ComputePathToPose plans on the map/costmap: a panel the robot never OBSERVED
# is in neither, validation succeeds, and the up-front lane never triggers.
# So the runner never inserts the blocker itself — the protocol is: map the
# house open, THEN close_partition.sh, THEN drive/look so the panel enters
# /map, return to the start pose, and only then run this script.
#   U-A: the partition coordinate must read FREE (a residual corrupts the
#        control exactly as it wedges a U-B run).
#   U-B: the partition coordinate must read OCCUPIED (panel present AND
#        observed), otherwise pre-flight would succeed and the trial would
#        silently measure nothing.
set +e
python3 "$EVAL_DIR/map_residual_check.py" --x "$BLOCKER_X" --y "$BLOCKER_Y"
RESIDUAL_RC=$?
set -e
if [ "$BLOCKER_PRESENT" = "True" ]; then
  if [ "$RESIDUAL_RC" = "0" ]; then
    echo "ABORT: the partition coordinate reads FREE in /map, so pre-flight"
    echo "validation would SUCCEED and the up-front lane would never trigger."
    echo "Insert and observe the panel first:"
    echo "  1. bash src/semantic_nav_bringup/scripts/close_partition.sh"
    echo "  2. teleop toward the gap until the panel shows in /map (59 deg FOV)"
    echo "  3. return to the start pose, kill teleop, retry"
    exit 1
  elif [ "$RESIDUAL_RC" = "3" ]; then
    echo "ABORT: could not read /map to confirm the panel is observed."
    exit 1
  fi
else
  if [ "$RESIDUAL_RC" = "2" ]; then
    echo "ABORT: residual obstacle in /map at the partition coordinate."
    echo "Relaunch and remap so the corridor reads free before this rep."
    exit 1
  elif [ "$RESIDUAL_RC" = "3" ]; then
    echo "WARN: could not read /map to verify the corridor is clear; proceeding."
  fi
fi

RUN_LOG=$(ls -t "$EVAL_DIR"/logs/*key.log | head -1)
[ -n "$RUN_LOG" ] || {
  echo "no *key.log found — is the stack logging?"
  exit 1
}
START_LINE=$(wc -l < "$RUN_LOG")

# A trial run on uncommitted code must never masquerade as a clean revision.
COMMIT=$(git -C "$EVAL_DIR/.." describe --always --dirty)
HEAD_COMMIT=$(git -C "$EVAL_DIR/.." rev-parse HEAD)
DIRTY_FILES=$(git -C "$EVAL_DIR/.." status --porcelain --untracked-files=no | wc -l)
DIFF_SHA256=$(git -C "$EVAL_DIR/.." diff --binary HEAD | sha256sum | awk '{print $1}')

echo "[TRIAL] scenario=$SCEN arm=$ARM rep=$REP commit=$COMMIT head=$HEAD_COMMIT dirty_files=$DIRTY_FILES diff_sha256=$DIFF_SHA256 start=$(date +%s)" \
  | tee "$OUT"

# NL command path: parse first (as navigation_terminal does), then navigate.
# /navigate_to_query rejects an empty query, so the parsed object_tag is the
# query and the raw NL command + intent hint ride along for traceability.
PARSE_OUT=$(ros2 service call /parse_semantic_command \
  semantic_nav_interfaces/srv/ParseSemanticCommand "{command: '$NLCMD'}")
echo "$PARSE_OUT" | tee -a "$OUT"

TAG=$(echo "$PARSE_OUT" | grep -o "object_tag='[^']*'" | head -1 | sed "s/object_tag='\(.*\)'/\1/")
HINT=$(echo "$PARSE_OUT" | grep -o "intent_hint='[^']*'" | head -1 | sed "s/intent_hint='\(.*\)'/\1/")
INTENT=$(echo "$PARSE_OUT" | grep -o "intent='[^']*'" | head -1 | sed "s/intent='\(.*\)'/\1/")

if [ "$INTENT" != "navigate_to_object" ] || [ -z "$TAG" ]; then
  echo "ABORT: LLM parse of '$NLCMD' did not yield a navigate intent" \
       "(intent='$INTENT', tag='$TAG'). Warm up the LLM and retry." | tee -a "$OUT"
  exit 1
fi

echo "[TRIAL] dispatch_wall=$(date +%s.%N)" >> "$OUT"

# Blocks until the pipeline finishes (success, escalation, or abort).
ros2 service call /navigate_to_query semantic_nav_interfaces/srv/NavigateToQuery \
  "{query: '$TAG', nl_command: '$NLCMD', intent_hint: '$HINT'}" \
  | tee -a "$OUT"

echo "[TRIAL] finish_wall=$(date +%s.%N)" >> "$OUT"
echo "[TRIAL] end=$(date +%s)" >> "$OUT"

# Slice the session key-log to this trial's window.
tail -n +"$((START_LINE + 1))" "$RUN_LOG" >> "$OUT"

echo "[TRIAL] wrote $OUT"
if [ "$BLOCKER_PRESENT" = "True" ]; then
  echo "[TRIAL] reminder: remove the panel (open_partition.sh) if it survived"
  echo "        this rep, and start the next rep from a FRESH SLAM session."
fi
