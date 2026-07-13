#!/usr/bin/env bash
# Usage: <command> 2>&1 | bash eval/log_session.sh <component>
# Example: ros2 launch ... 2>&1 | bash eval/log_session.sh system
#
# Writes TWO timestamped logs to eval/logs/ while still printing everything
# to the terminal:
#   <component>_YYYYMMDD_HHMMSS.log      full session (keep: debugging needs
#                                        planner/behavior_server/rtabmap lines)
#   <component>_YYYYMMDD_HHMMSS.key.log  semantic-stack lines only — the
#                                        orchestrator/navigator/resolver/
#                                        validator/executor output that the
#                                        eval artifacts are built from
#
# Override the filter with LOG_KEY_PATTERN (awk ERE), e.g.:
#   LOG_KEY_PATTERN='\[UP_FRONT\]' ros2 launch ... 2>&1 | bash eval/log_session.sh system

COMPONENT=${1:-"unknown"}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"

mkdir -p "$LOG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/${COMPONENT}_${TIMESTAMP}.log"
KEY_FILE="$LOG_DIR/${COMPONENT}_${TIMESTAMP}.key.log"

# All output of the semantic stack's own nodes; Nav2/RTAB-Map/Gazebo noise
# stays in the full log only.
PATTERN=${LOG_KEY_PATTERN:-'^\[(navigation_orchestrator|navigator_node|resolver_node|validator_node|executor_node|mock_map_provider)'}

echo "[log_session] Writing $COMPONENT full log to: $LOG_FILE" >&2
echo "[log_session] Writing $COMPONENT key log  to: $KEY_FILE" >&2

exec tee -a "$LOG_FILE" | awk -v key="$KEY_FILE" -v pat="$PATTERN" \
    '{ print; fflush(); if ($0 ~ pat) { print >> key; fflush(key) } }'
