# Up-Front Ablation — Runbook

Companion to `upfront_evaluation_plan.md`. The harness (§3) is built:
`upfront_scenarios.yaml`, `upfront_ablation.py` (parser, fixture-tested),
`run_upfront_trial.sh` (all pre-flight gates), `test_upfront_harness.py`.
This file is the operational, terminal-by-terminal procedure.

## Run matrix (the 4-hour variant)

| Scenario | Arms | Reps | Runs | Order |
|---|---|---|---|---|
| U-B open-set barrier | U2 → U1 → U0 | 5 | 15 | **first** |
| U-A control | U2, U1, U0 | 3 | 9 | after |

**Run U-B/U2 rep 1 before anything else.** If it does not reproduce the
demonstration run, stop and reassess instead of burning 4 h.

Group reps by arm: U0 and non-U0 need different launches (see below), and
`up_front_recovery_enabled` cannot be flipped live.

## One-time per session

Every terminal: rover env, i.e. `source ~/ugv_sim_env.bash` (which must chain
the Gazebo env — `source /usr/share/gazebo/setup.sh` BEFORE the
`GAZEBO_MODEL_PATH` colon-append, or gzserver SIGABRTs).

1. **Zenoh bridge** (exactly ONE, with the config):
   `./zenoh-bridge-ros2dds -c zenoh-local.json5`
2. **LLM**: llama_ros with **Qwen3-14B** (matches the en-route grid), plus
   `navigator_node` — started manually as always. Exactly one navigator alive.
3. **navigation_terminal** in its own TTY (serves `/operator_decision`):
   `ros2 run semantic_nav_orchestrator navigation_terminal --ros-args -p use_sim_time:=true`

## Launch, per arm

Piped through the session logger — the runner slices the newest `*key.log`.

**U1 / U2** (defaults; the runner flips `open_set_inference_enabled` live):

```bash
ros2 launch semantic_nav_bringup aws_small_house_ugv_semantic.launch.py \
  2>&1 | bash eval/log_session.sh upfront
```

**U0** (both overrides are REQUIRED — with up-front recovery off the goal
dispatches into the Nav2 BT, and the default LLM tree would rescue the
baseline; the runner aborts if either is missing):

```bash
ros2 launch semantic_nav_bringup aws_small_house_ugv_semantic.launch.py \
  up_front_recovery_enabled:=false \
  recovery_bt_xml:=$(ros2 pkg prefix semantic_nav_nav2_plugins)/share/semantic_nav_nav2_plugins/config/semantic_recovery_bt_geometric.xml \
  2>&1 | bash eval/log_session.sh upfront
```

## Per-repetition cycle (~10 min reset + ~160 s trial)

The blocker must be **observed into `/map` before the trial** — the pre-flight
`ComputePathToPose` plans on the map/costmap, and a panel the robot never saw
is in neither, so validation would succeed and the up-front lane would never
trigger. The runner does NOT insert the blocker; steps 2–3 do.

1. **Fresh SLAM session** — relaunch the stack (no saved map). Map the house
   with the corridor **OPEN** (do the initial mapping WITHOUT the partition —
   mapping the doorway closed from the start bakes it into every graph node:
   the baked-door gotcha). The mapping drive must reach the **bedroom/bed
   region** (live SLAM: an unmapped goal fails validation by design). Use
   `eval/auto_map.sh` or teleop.
2. **Insert the partition (U-B only)** after mapping:
   `bash src/semantic_nav_bringup/scripts/close_partition.sh`
3. **Observe it (U-B only)**: teleop toward the gap until the panel shows up
   in `/map` (59° FOV — it must actually be in view). Verify:
   `python3 eval/map_residual_check.py --x -2.5068 --y -1.3503`
   should report a RESIDUAL (that is the panel, observed). Then return the
   robot near the start pose.
4. **Kill teleop.** A stale `teleop_twist_keyboard` publishes zero twists that
   silently poison the trial. The runner gates on this, but kill it now.
5. **Warm-up LLM call** after any relaunch (first token is slow):
   `ros2 service call /parse_semantic_command semantic_nav_interfaces/srv/ParseSemanticCommand "{command: 'tired'}"`
6. **Run the trial**:
   `bash eval/run_upfront_trial.sh U-B U2 1`
   The runner enforces, in order: cmd_vel publisher set; `/operator_decision`
   live (U-B); arm params match the launch (aborts with the fix if not); the
   blocker-state gate — U-A: partition coordinate reads FREE; U-B: it reads
   OCCUPIED (panel present and observed). Then it parses the NL command
   `tired` and dispatches `/navigate_to_query`, blocking until the terminal
   outcome.
7. **Operator action (U-B/U2 only).** When `navigation_terminal` prompts for
   `open_door_then_replan`: **remove the partition first** —
   `bash src/semantic_nav_bringup/scripts/open_partition.sh` — wait your
   fixed response interval (pick one — e.g. 10 s — and keep it constant;
   note it, it enters time-to-resolution), then confirm at the terminal.
   The rover's dwell re-observes the vacated gap and `plan_ok` flips.
   U-B/U1 and U-B/U0 end in escalation / needs-operator: **do not** remove
   the panel; just let the run terminate.
8. **Teardown** (Ctrl-C the launch) and go to 1 for the next rep. If the
   panel survived the rep (U1/U0), `open_partition.sh` before teardown keeps
   Gazebo clean, but the fresh relaunch is what actually resets state.

U-A reps are the same cycle minus steps 2–3 and the operator step.

## Discard rules (pre-declared; record every discard)

Re-run only on: absent `/operator_decision` server, stray `/cmd_vel`
publisher, residual-map gate failure, or the NL parse refusing to produce a
navigate intent (cold LLM). Anything else is a result, not a discard.

## After the campaign

```bash
python3 eval/upfront_ablation.py     # -> eval/upfront_ablation_results.csv
```

Read the CSV against plan §5. The load-bearing columns:

- `upfront_recovery_triggered` — must be False on every U-A row (all arms)
  and on U-B/U0; True on U-B/U1 and U-B/U2.
- `open_door_ever_eligible` + `escalated` — the structural claim: U1
  escalates with eligibility **False** (never eligible), never
  proposed-and-rejected. `raw_directive` vs `accepted_directive` stay
  separate so containment remains measurable.
- `original_goal_revalidated`, `navigation_success`,
  `original_target_preserved` — the U2 success chain. `plan_ok` is the real
  gate; there is deliberately no barrier-clear column (known near-no-op —
  never cite it).
- `enroute_recovery_fired` — flags contamination by a post-dispatch en-route
  intervention (the 14B reference trace had one); report such runs honestly.

Then the ~20 min of chapter edits in plan §6.
