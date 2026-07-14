# CLAUDE.md

ROS 2 Humble **semantic navigation stack** with an optional LLM front-end. RTAB-Map for live SLAM, Nav2 for planning/execution, AWS RoboMaker Small House world. Authoritative user-facing docs are in `README.md`.

## TWO ROBOTS — know which one you are touching

| | TurtleBot3 Waffle | **Waveshare UGV Rover** |
|---|---|---|
| launch | `semantic_nav_system.launch.py` | `aws_small_house_ugv_semantic.launch.py` |
| nav2 params | `nav2_semantic_params.yaml` | `rover_semantic_nav_params.yaml` |
| SLAM launch | `rtabmap_demos` (third-party) | `rover_rtabmap_rgbd.launch.py` (ours) |
| sensing | 360° LiDAR | **depth-only by default** (no LiDAR, 59° FOV) |
| status | **frozen** — holds the locked Ch7 LLM-side numbers | **the platform for BT-led recovery evaluation** (decided 2026-07-13) |

**The rover is the deployment target** (Jetson Orin Nano + OAK-D Lite, no LiDAR), so
recovery results are collected on it. `depth_only:=false` restores a simulated LiDAR for
comparison only.

**`semantic_nav_orchestrator` is SHARED by both.** Its parameter defaults reproduce TB3's
original behaviour byte-for-byte (`up_front_reobserve_mode='spin'`, 10 s allowance, the
original `(+y,-2y,+y)` sweep). The rover **opts in** to new behaviour from its own launch —
never change a shared default for the rover's benefit, and check
`git diff origin/main -- <tb3 files>` is empty before committing rover work.

Rover failure modes and fixes: **`ugv_rover_depth_only_issues.md`** (read it before
debugging rover SLAM/nav — the obvious diagnosis is usually wrong).

## Workspaces (sourced in this order)

1. `~/demo_bringup` — third-party deps:
   - `src/rtabmap_ros`, `src/rtabmap` (built)
   - `src/llama_ros` (built; provides `llama_msgs` + `/llama/generate_response`)
   - `aws-robomaker-small-house-world/` as a **sibling** of `src/`, asset-only, NOT built
2. `~/ugv_ws` — Waveshare `ugv_ws` (rover model, `ugv_gazebo`, `ugv_description`). Rover only.
3. `~/semantic_nav_ws` — this repo

Rover env helper: `~/ugv_sim_env.bash`.

Build order: `~/demo_bringup` → `~/semantic_nav_ws`. Sourcing `~/demo_bringup/install/setup.bash` makes both RTAB-Map and `llama_ros` available.

If `llama_ros` is not present, build this workspace with `--packages-skip semantic_nav_llm`.

## Pipeline

```
(optional) NL command
  → semantic_nav_llm/navigator_node          (llama_ros + GBNF → {action,target,confidence})
  → semantic_nav_semantics/resolver_node     (name → PoseStamped + db_version/db_stamp)
  → semantic_nav_validator/validator_node    (Nav2 ComputePathToPose feasibility check)
  → semantic_nav_executor/executor_node      (bridges custom action to Nav2 NavigateToPose)
```

`semantic_nav_orchestrator/navigation_orchestrator` runs the resolve → validate → execute pipeline as a one-shot and propagates `db_version` / `db_stamp` through every stage's log line. Stage prefixes: `[LLM_INTENT]`, `[RESOLUTION]`, `[VALIDATION]`, `[EXECUTION]`.

## Packages (src/)

| Package | Type | Role |
|---|---|---|
| `semantic_nav_interfaces` | ament_cmake | msgs/srvs/actions (`ResolveLocation`, `ValidatePose`, `ParseSemanticCommand`, `ExecutePose`, `SemanticDB`) |
| `semantic_nav_semantics` | ament_python | `resolver_node`, `SemanticStore` (immutable snapshot, atomically swapped on live `/semantic_map/updates`), `mock_map_provider_node` |
| `semantic_nav_validator` | ament_python | `validator_node` (Nav2 ComputePathToPose wrapper) |
| `semantic_nav_executor` | ament_python | `executor_node` (`ExecutePose` action → Nav2 `NavigateToPose`) |
| `semantic_nav_orchestrator` | ament_python | `navigation_orchestrator` (one-shot pipeline) |
| `semantic_nav_llm` | ament_python | `navigator_node` (NL → GBNF-constrained intent JSON; consumes `/llama/generate_response`) |
| `semantic_nav_bringup` | ament_python | `semantic_nav_system.launch.py` (Gazebo + RTAB-Map + Nav2 + RViz + semantic core nodes) |

## Key conventions

- **All poses are in the `map` frame.** The resolver rejects DB entries with any other `frame_id`; the orchestrator's `_pose_is_valid_for_navigation` rejects non-finite values, empty/wrong frames, and near-zero quaternions.
- **Live SLAM, no saved map.** Targets must already be in mapped free space at query time — otherwise validation fails. This is by design.
- **No LLM → motion shortcut.** `navigator_node` only emits structured intent JSON; it cannot fabricate poses, velocities, or unknown rooms. Targets are validated against `semantic_db.json` (with normalization + leading-article stripping) before being accepted.
- **GBNF-enforced LLM output.** Grammar at `src/semantic_nav_llm/config/semantic_intent.gbnf` constrains the LLM to `{"action": "navigate|clarify|reject", "target": "string", "confidence": 0-100}`. Confidence floor (default 60) gates `navigate` acceptance.
- **`db_version` / `db_stamp` are propagated end-to-end** so any single run can be traced back to a specific DB state. Live object-centric ingestion IS wired up: `resolver_node` subscribes to `/semantic_map/updates` (`SemanticMapUpdate`, TRANSIENT_LOCAL) and `/semantic_store_updated`, atomically swapping its snapshot and bumping `db_version` at runtime. The live source is `mock_map_provider_node` (a static-JSON stand-in for a ConceptGraph-style detector).
- **CLI shorthand for the orchestrator**: positional args become the query, e.g. `ros2 run semantic_nav_orchestrator navigation_orchestrator living room`.

## Where things live

- Semantic DB: `src/semantic_nav_semantics/config/semantic_db.json` (coordinates are AWS-Small-House-specific).
- GBNF grammar: `src/semantic_nav_llm/config/semantic_intent.gbnf`.
- Bringup launch: `src/semantic_nav_bringup/launch/semantic_nav_system.launch.py` (does **not** launch `navigator_node` or `llama_ros` — both are started manually).
- Long-form project history/decisions: `semantic_nav_project_context.md` at repo root.

## Required env vars (every terminal)

```
TURTLEBOT3_MODEL=waffle
source /usr/share/gazebo/setup.sh          # sets GAZEBO_RESOURCE_PATH / GAZEBO_PLUGIN_PATH
TURTLEBOT3_GAZEBO_MODELS=/opt/ros/humble/share/turtlebot3_gazebo/models
AWS_SMALL_HOUSE=$HOME/demo_bringup/aws-robomaker-small-house-world
GAZEBO_MODEL_PATH=$AWS_SMALL_HOUSE/models:$TURTLEBOT3_GAZEBO_MODELS:$GAZEBO_MODEL_PATH
```

`source /usr/share/gazebo/setup.sh` must come **before** the `GAZEBO_MODEL_PATH` line (it also sets that var, so colon-append `:$GAZEBO_MODEL_PATH` to keep our model paths). Without it `GAZEBO_RESOURCE_PATH` is unset and gzserver **aborts with SIGABRT** ("Unable to find shader lib" → "Failed to initialize scene") the moment the Waffle's camera/lidar loads — the symptom is a bringup that spawns nothing (not even the world). Note `/opt/ros/humble/setup.bash` does NOT set `GAZEBO_RESOURCE_PATH`; only Gazebo's own setup.sh does. Reinstalling `ros-humble-gazebo-*` resets this. See `reference_gazebo_resource_path_gotcha` memory.

`GAZEBO_MODEL_PATH` must be **colon-joined** (not reassigned with `=`) so the AWS furniture meshes resolve — a previous `.bashrc` line using `=` is the most common cause of an empty Gazebo house.

## Out of scope when changing things

- Don't reintroduce the old **location-centric** `SemanticDB.msg` topic subscriber in `resolver_node` — it was reverted intentionally. Note this is NOT the live-update path: the current object-centric live ingestion uses `SemanticMapUpdate` (`/semantic_map/updates`) and `SemanticStoreUpdated` (`/semantic_store_updated`), which ARE wired and supported (see `project_semantic_db_versioning` memory).
- Don't add a third workspace for `llama_ros`; it lives in `~/demo_bringup/src` together with RTAB-Map.
- Don't pre-bake a map. RTAB-Map is online; teleop is the standard way to explore before issuing queries.

## Rover: non-obvious facts (do not re-derive these)

- **Sim odom is GROUND TRUTH.** `gazebo_ros_diff_drive` defaults `odometry_source=WORLD`
  (verified 0.0° / 0.000 m through a full spin). So RTAB-Map gets a perfect motion prior and
  **sim SLAM is an optimistic upper bound, not a hardware preview.** On the Jetson wheel odom
  drifts and the IMU genuinely matters; in sim the IMU is wired to nothing
  (`Optimizer/GravitySigma=0`).
- **Skid-steer**: 4 fixed wheels. Delivers only **~0.47× commanded rotation** and **coasts
  7–17°** after `cmd_vel=0`. Two attempts to tune the coast away both FAILED
  (`max_wheel_acceleration` raised 20× = no change; raising smoother decel made it *worse*).
  Treat the coast as a platform fact.
- **rtabmap declares every `Grid/*` param as a STRING.** A bare `LaunchConfiguration` gets
  auto-converted to double/bool and rtabmap **SIGABRTs before printing anything**. Use
  `ParameterValue(v, value_type=str)`.
- **`map_always_update=false` (rtabmap default) freezes the map while the robot is parked** —
  the grid only refreshes on a new graph node (≥0.1 m / ≥0.1 rad of motion). That, not
  perception, is why recovery used to spin. We set it true, so the rover re-observes a cleared
  doorway **without moving**.
- **Clearing an obstacle needs a ray to pass THROUGH the cell and terminate on something**
  within `Grid/RangeMax`. Rays into open space return NaN and trace nothing — such an obstacle
  can never be cleared, however long you dwell or spin.
- The rover drives behind a **`RotationShimController`** (turns in place before committing to a
  path). It must be paired with **`PoseProgressChecker`** — `SimpleProgressChecker` counts only
  translation and aborts every sharp turn.
- `.gitignore` has a blanket `models/` rule (aimed at LLM weights): Gazebo model dirs must be
  `git add -f`'d. `models/door_scenario/` is still untracked because of this.

## Rover: status (2026-07-14) — a full BT-led recovery now runs end to end

`"tired"` → bed → pre-flight blocked → affordance inferred → LLM `approach_and_recheck` →
standoff → LLM `open_door_then_replan` → operator opens → **dwell (no motion)** → `plan_ok` →
drives → `SUCCEEDED`. The three former blockers are resolved or explained.

**PRE-FLIGHT BEFORE ANY TRIAL — non-negotiable:**
```bash
ros2 topic info /cmd_vel -v | grep "Publisher count"   # velocity_smoother + behavior_server ONLY
```
A stale `teleop_twist_keyboard` publishes **zero twists at 10 Hz** onto `/cmd_vel` and races the
velocity smoother; `gazebo_ros_diff_drive` has **no cmd_vel timeout**, so it votes forever. The
rover then stalls and it looks *exactly* like a controller/tuning bug. This has cost a full
debugging detour three times.

### Two bugs that both masquerade as "the rover can't fit through the narrow gap"

The gap is innocent — measured 0.95–1.00 m in `/map`, 0.75–0.80 m of costmap corridor, and the
rover is 0.2 m wide. TB3 is **2.2× fatter** (`robot_radius` 0.22 vs 0.1) with the same inflation
and threads the same door fine.

1. **DWB could not TURN.** `acc_lim_theta: 0.5` (Waveshare leftover) caps DWB's reachable
   velocity window at ±0.05 rad/s at 10 Hz → it commands ~0.02 rad/s, the skid-steer
   under-delivers, and the rotation can never ramp up. Fixed → 3.2 / −3.2.
2. **NavFn could not extract a path.** `Failed to create a plan from potential when a legal
   potential was found` means a path **provably exists** and the gradient descent stalled: the
   59° depth FOV leaves unknown cells scattered in open floor, and NavFn maps unknown → cost
   **253** (free = 50), filling the potential field with local minima. Fixed with
   **`SmacPlanner2D`** (real A*, no gradient descent). **TB3 keeps NavFn** — its 360° LiDAR
   leaves no such holes.

### Known no-op, deliberately left as-is

**The barrier-clear gate always returns `cleared`.** A 1-cell-thick door reaches at most
11/121 = **0.091** lethal fraction against a **0.15** threshold — `still_blocked` is
mathematically unreachable. `plan_ok` does 100% of the real gating. Runs still succeed, but
**never cite "the robot confirmed the barrier was clear" as trial evidence.** Any fix must be
scale-invariant (`clear_radius` grows with `barrier_extent_m`, so a plain lethal-cell count also
breaks — tried and reverted).

### Still open

- **Removed obstacles REAPPEAR in `/map`** after a graph optimization — RTAB-Map re-asserts old
  per-node local grids. The barrier lives in `/map`, NOT the costmap. Fix (verified live, **not
  wired**): call `/rtabmap/cleanup_local_grids` *while the map still shows the cell empty* — it
  is a no-op once the map re-seals.
- **RTAB-Map approx-sync starvation**: `/odom` at ~100 Hz vs `topic_queue_size: 10` → bringup
  hangs with no `/map` and Nav2 stuck `inactive`.
- The rover can end up facing away from the blocker after the dwell (59° FOV).
