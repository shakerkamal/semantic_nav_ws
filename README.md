# LLM Navigator

A modular ROS 2 Humble semantic navigation stack that lets a robot drive to semantically named objects (e.g. `"bed"`, `"couch"`) instead of raw coordinates, and accepts free-form natural-language commands (e.g. `"I am hungry"`) that an LLM translates into a constrained semantic intent. Built on top of RTAB-Map for online SLAM and Nav2 for planning/execution, running a TurtleBot3 Waffle in the **AWS RoboMaker Small House** Gazebo world (chosen because RTAB-Map's visual loop closure needs a texture-rich environment).

---

## Table of contents

1. [What this project does](#1-what-this-project-does)
2. [System architecture](#2-system-architecture)
3. [Required workspaces and packages](#3-required-workspaces-and-packages)
4. [First-time setup](#4-first-time-setup-step-by-step)
5. [Building](#5-building)
6. [Environment variables](#6-environment-variables-critical)
7. [Running the system](#7-running-the-system)
8. [Navigation terminal](#8-navigation-terminal)
9. [Natural-language commands via the LLM navigator](#9-natural-language-commands-via-the-llm-navigator)
10. [Semantic database](#10-semantic-database)
11. [Launch arguments reference](#11-launch-arguments-reference)
12. [ROS 2 endpoints](#12-ros-2-endpoints)
13. [Interface definitions](#13-interface-definitions)
14. [Troubleshooting](#14-troubleshooting)
15. [Design principles](#15-design-principles)
16. [Roadmap](#16-roadmap)

---

## 1. What this project does

Given a semantic object query (`chair`, `refrigerator`) or a natural-language command (e.g. `"I am hungry"`) that an LLM converts into one, the system:

1. **Parses** (optional) a natural-language command via `semantic_nav_llm` into a constrained `{action, object_tag, intent_hint, confidence}` JSON intent enforced by a GBNF grammar.
2. **Resolves** the object tag to a `PoseStamped` in the `map` frame using an object-centric semantic database (`map_v001.json`, N objects), ranked by Hybrid(Deterministic+LLM) caption scoring, and stamps the response with `db_version`/`db_stamp`.
3. **Validates** that Nav2's planner can currently compute a path to that pose via `ValidateSemantic` inside the Nav2 behavior tree.
4. **Executes** the navigation via Nav2's `NavigateToPose`, using a custom Nav2 behavior tree (`semantic_recovery_bt.xml`) with a three-tier recovery strategy on path failure.
5. **Recovers** (if blocked) using a three-tier strategy owned entirely by the Nav2 BT:
   - **Tier 1 — continuous replanning:** `RateController(1 Hz)` around `ComputePathToPose` replans against the current costmap every second. Small navigable obstacles are routed around automatically; no recovery fires.
   - **Tier 2 — geometric recovery:** `ClearEntireCostmap` (local + global) followed by `BackUp`. Resolves stale costmap data and transient stuck situations.
   - **Tier 3 — semantic recovery:** `CaptureBlockageContext` samples the last path against the live costmap to locate the blockage centroid, then `QuerySemanticContext` identifies the responsible object, `EscalateToLLMRecovery` calls `/request_recovery`, and the orchestrator issues a directive (`retry_target`, `wait_then_replan`, `give_up`) with up to 3 retries.

Each stage gates the next: if parsing/resolution fails, downstream stages are skipped. RTAB-Map provides live SLAM (no pre-saved map), so semantic destinations must already be in the explored, navigable free space at the time you issue the query.

---

## 2. System architecture

### Pipeline

```
User intent (typed in navigation_terminal)
    │
    ▼  NL path: parse via LLM → object_tag / object_key
    │  Direct path: object key (e.g. "chair:2") used as-is
    ▼
ResolveLocation  (object_tag → Hybrid-ranked ObjectRow → PoseStamped + db_version/db_stamp)
    │
    ▼
ExecutePose  (Nav2 NavigateToPose with semantic_recovery_bt.xml)
    │
    │   ┌── PRIMARY ─────────────────────────────────────────────────────────┐
    │   │  ValidateSemantic  (geometric veto before motion)                  │
    │   │  RateController(1 Hz) → ComputePathToPose  (continuous replanning) │
    │   │  FollowPath                                                         │
    │   └────────────────────────────────────────────────────────────────────┘
    │         │ FollowPath fails
    │   ┌── RECOVERY (RoundRobin) ───────────────────────────────────────────┐
    │   │  Tier 2 — GeometricSafeRecovery:                                   │
    │   │    ClearLocalCostmap + ClearGlobalCostmap + BackUp                 │
    │   │                                                                     │
    │   │  Tier 3 — SemanticRecoveryBranch:                                  │
    │   │    CaptureBlockageContext (blockage_centroid / extent)              │
    │   │    → QuerySemanticContext (responsible object lookup)               │
    │   │    → EscalateToLLMRecovery → /request_recovery                     │
    │   │    → directive: retry_target | wait_then_replan | give_up          │
    │   │         operator actions: open_door | clear_object                 │
    │   └────────────────────────────────────────────────────────────────────┘
    ▼
Robot moves (or graceful give_up after 3 BT retries)
```

### Package layout (this workspace, `src/`)

| Package | Type | Responsibility |
|---|---|---|
| `semantic_nav_interfaces` | ament_cmake | Custom msgs/srvs/actions (`ResolveLocation`, `ValidatePose`, `ExecutePose`, `RequestRecovery`, `ProposeRecovery`, `MatchResponsibleObject`, `RefreshLocalObjects`, `NavigateToQuery`, `OperatorDecision`, `ObjectInstance`, `RecoveryTrigger`) |
| `semantic_nav_semantics`  | ament_python | Object-centric semantic resolution: `SemanticStore` (immutable `map_v001.json` snapshot), `resolver_node`, `local_object_query_node`, `StandoffPlanner`, BM25/LLM/Hybrid caption rankers, `SpatialContextBuilder` |
| `semantic_nav_validator`  | ament_python | Path-existence check via `ComputePathToPose` |
| `semantic_nav_executor`   | ament_python | Bridges custom `ExecutePose` action to Nav2 `NavigateToPose` |
| `semantic_nav_orchestrator` | ament_python | Idle service daemon: serves `/navigate_to_query`, `/cancel_navigation`, `/request_recovery`, `/match_responsible_object`; LLM-backed directive generation; launched from bringup. Also provides `navigation_terminal` — the unified interactive CLI |
| `semantic_nav_llm`        | ament_python | NL → constrained semantic intent (llama_ros + GBNF); serves `/parse_semantic_command` and `/propose_recovery` |
| `semantic_nav_nav2_plugins` | ament_cmake (C++) | Nav2 BT plugins: `ValidateSemantic`, `PathClearCondition` (with severity gating + `BlockageMetrics`), `CaptureBlockageContext`, `QuerySemanticContext`, `EscalateToLLMRecovery`, `EmitObstacleSignal`, `OperatorPrompt`; BT XML: `semantic_recovery_bt.xml` |
| `semantic_nav_operator_io` | ament_python | Serves `/operator_decision` via stdin. **Not launched by default** — `navigation_terminal` handles operator prompts inline. Set `enable_operator_io:=true` for headless/CI use |
| `semantic_nav_bringup`    | ament_python | Launch files for the integrated stack |

---

## 3. Required workspaces and packages

This project lives across **two workspaces** that both need to be sourced at runtime:

| Workspace | Holds | Built? |
|---|---|---|
| `~/demo_bringup` | RTAB-Map ROS 2 sources (`rtabmap_ros`, `rtabmap`) and `llama_ros` sources inside `src/`; `aws-robomaker-small-house-world/` cloned as a sibling of `src/` | RTAB-Map and `llama_ros` yes (colcon). AWS world **no** - we only consume its `worlds/` and `models/` assets |
| `~/semantic_nav_ws` | **This repo** - all `semantic_nav_*` packages | Yes |

> If you do not have `demo_bringup` yet, follow [Section 4](#4-first-time-setup-step-by-step) below to create it.

### System dependencies (apt)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-simulations \
  ros-humble-turtlebot3-gazebo \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-gazebo-ros-pkgs \
  python3-colcon-common-extensions \
  python3-rosdep \
  git
```

If `rosdep` has never been initialized on this machine:

```bash
sudo rosdep init      # ignore "already exists" warnings
rosdep update
```

---

## 4. First-time setup (step by step)

These steps assume you have Ubuntu 22.04 and ROS 2 Humble installed. If you have never sourced ROS 2 before, run `source /opt/ros/humble/setup.bash` once in any new terminal you open from now on - every command below assumes ROS 2 is sourced.

### 4.1 Clone this repo

```bash
mkdir -p ~/semantic_nav_ws/src
cd ~/semantic_nav_ws/src
git clone <THIS_REPO_URL> .
```

### 4.2 Set up `demo_bringup` (RTAB-Map + llama_ros sources + AWS world assets)

```bash
mkdir -p ~/demo_bringup/src

# RTAB-Map sources - will be built with colcon
cd ~/demo_bringup/src
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git
git clone https://github.com/introlab/rtabmap.git

# llama_ros sources - will be built with colcon (only needed for the LLM navigator).
git clone https://github.com/mgonzs13/llama_ros.git

# AWS small house - cloned as a sibling of src/, NOT built
cd ~/demo_bringup
git clone --branch ros2 https://github.com/aws-robotics/aws-robomaker-small-house-world.git
```

The AWS small house package is **not** a buildable ROS dependency for our setup - we only use its `worlds/small_house.world` file and `models/` directory. Cloning it outside `src/` keeps colcon from trying to build it, which avoids pulling in `gazebo_ros` as a build-time dependency and keeps the workspace clean.

Follow the upstream `llama_ros` README for any extra apt/pip prerequisites it needs (e.g. CUDA toolkit for GPU acceleration). You will also need a GGUF model file on disk (LLaMA 3 8B Instruct GGUF is the model this project was developed against) and a `llama_ros` launch configuration that points at it.

### 4.3 Patch the TurtleBot3 Waffle SDF (required by RTAB-Map)

The stock TurtleBot3 Waffle has an RGB camera. RTAB-Map's demo expects a **depth** camera with an optical frame. Create a backup of the `model.sdf` file and then edit
`/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf`:

1. Change the camera sensor type from `<sensor name="camera" type="camera">` -> `<sensor name="camera" type="depth">`.
2. Change image resolution from `1920x1080` -> `640x480`.
3. Rename `<link name="camera_rgb_frame">` -> `<link name="camera_rgb_optical_frame">`.
4. Add a new empty `<link name="camera_rgb_frame"/>`.
5. Add the optical joint:
   ```xml
   <joint name="camera_rgb_optical_joint" type="fixed">
     <parent>camera_rgb_frame</parent>
     <child>camera_rgb_optical_frame</child>
     <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
     <axis><xyz>0 0 1</xyz></axis>
   </joint>
   ```
6. (Optional, recommended) Increase the lidar min range from `0.12` -> `0.2` to avoid self-hits.

The same patch is documented in the header comment of `/opt/ros/humble/share/rtabmap_demos/launch/turtlebot3/turtlebot3_sim_rgbd_scan_demo.launch.py` - refer to it if anything is unclear.

---

## 5. Building

Build the two workspaces **in this order** (`demo_bringup` first, then this repo):

```bash
# 1. demo_bringup builds both rtabmap_* and llama_ros from src/
source /opt/ros/humble/setup.bash
cd ~/demo_bringup
rosdep install -i --from-path src -y --rosdistro humble
colcon build --symlink-install

# 2. This workspace
source /opt/ros/humble/setup.bash
source ~/demo_bringup/install/setup.bash
cd ~/semantic_nav_ws
rosdep install -i --from-path src -y --rosdistro humble
colcon build --symlink-install
```

The `aws-robomaker-small-house-world/` directory at `demo_bringup/` is intentionally outside `src/`, so colcon will ignore it. If any build fails with "package not found", you almost certainly forgot to source `demo_bringup/install/setup.bash` before building this workspace.

---

## 6. Environment variables (critical)

Four variables must be set before any launch, in **every terminal** you use:

```bash
export TURTLEBOT3_MODEL=waffle
export TURTLEBOT3_GAZEBO_MODELS=/opt/ros/humble/share/turtlebot3_gazebo/models
export AWS_SMALL_HOUSE=$HOME/demo_bringup/aws-robomaker-small-house-world
export GAZEBO_MODEL_PATH=$AWS_SMALL_HOUSE/models:$TURTLEBOT3_GAZEBO_MODELS
```

For convenience, add this block to the **end** of `~/.bashrc`:

```bash
# --- semantic_nav: env ---
export TURTLEBOT3_MODEL=waffle
export TURTLEBOT3_GAZEBO_MODELS=/opt/ros/humble/share/turtlebot3_gazebo/models
export AWS_SMALL_HOUSE=$HOME/demo_bringup/aws-robomaker-small-house-world
export GAZEBO_MODEL_PATH=$AWS_SMALL_HOUSE/models:$TURTLEBOT3_GAZEBO_MODELS
# --- end semantic_nav ---
```

**Why `GAZEBO_MODEL_PATH` matters:** the AWS world references its 60+ furniture meshes via `model://aws_robomaker_residential_*` URIs. Gazebo resolves those URIs by scanning `GAZEBO_MODEL_PATH`. The AWS world is asset-only (not built), so nothing exports `GAZEBO_MODEL_PATH` for you - you must set it manually or the Gazebo window opens empty and you get dozens of `Error Code 12 Msg: Unable to find uri[model://...]` errors.

**Common pitfall:** if your `.bashrc` already has a line like `export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models` (using `=`, not appending), it overwrites the variable every time a new shell starts and clobbers any AWS path you add. Replace any such line with the colon-joined version above, then open a fresh terminal and verify:

```bash
echo $GAZEBO_MODEL_PATH
# Expected: both paths joined with a colon
```

---

## 7. Running the system

### Terminal 1 — full system bringup

```bash
source /opt/ros/humble/setup.bash
source ~/demo_bringup/install/setup.bash
source ~/semantic_nav_ws/install/setup.bash

ros2 launch semantic_nav_bringup semantic_nav_system.launch.py
```

### What starts and when

| t (s) | What starts |
|---|---|
| 0 | Gazebo (gzserver + gzclient), robot_state_publisher, TurtleBot3 Waffle spawn |
| 0 | `resolver_node`, `validator_node`, `executor_node`, `local_object_query_node` |
| 0 | `navigator_node` (LLM intent parser) — disable with `enable_llm:=false` |
| 0 | `operator_io_node` — **disabled by default** (`enable_operator_io:=false`); `navigation_terminal` handles operator prompts instead |
| 3 | RTAB-Map (`turtlebot3_rgbd_scan.launch.py`) |
| 5 | Nav2 (`navigation_launch.py`) and RViz (with `respawn=true` — a RViz crash no longer brings down the stack) |
| 10 | `navigation_orchestrator` — long-running idle service daemon; accepts `/navigate_to_query` calls from the terminal |

You should see Gazebo render the textured house, then RViz appear with a growing occupancy grid as RTAB-Map builds the map.

### Driving the robot manually (to explore the map)

Open a **second** terminal and run:

```bash
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Drive the robot through rooms until the area containing your target object is visible in RViz's occupancy grid. Semantic navigation will fail to validate goals in unexplored space.

### Terminal 2 — LLM model server (only if using NL commands)

See [Section 9.1](#91-start-the-llamaros-action-server) for `llama_node` startup instructions. This terminal uses the `llama_ros` virtual environment and should be kept separate from the ROS environment.

---

## 8. Navigation terminal

The `navigation_terminal` is the unified user interface. It replaces both the old one-shot orchestrator CLI and the separate `operator_io_node`. Start it after bringup is up:

```bash
source /opt/ros/humble/setup.bash
source ~/demo_bringup/install/setup.bash
source ~/semantic_nav_ws/install/setup.bash

ros2 run semantic_nav_orchestrator navigation_terminal
```

You will see a banner and an interactive prompt:

```
╔══════════════════════════════════════════════════════╗
║        Semantic Navigation Terminal                  ║
╚══════════════════════════════════════════════════════╝
  Type an object key (chair:2) or NL command (go to the kitchen).
  Type a new command at any time to cancel and reroute.
  Ctrl-C cancels active navigation.  Ctrl-D exits.

[nav] >
```

### 8.1 Command types

| What you type | How it is handled |
|---|---|
| `chair:2` | Recognised as a direct object key (`<tag>:<id>`); sent to the orchestrator immediately — no LLM call |
| `go to the kitchen` | Sent to `/parse_semantic_command` (LLM); resolved target is then navigated to |
| `refrigerator` | Sent to LLM; if it maps to a known object tag, that tag is used for resolution |

The terminal prints each stage as it happens:

```
[nav] > chair:2

  → Direct key: chair:2
  → Navigating to chair:2 …
  · EXECUTING
  · EXECUTING

  ✓ SUCCESS — reached chair:2

[nav] > go to the kitchen

  → Parsing NL: "go to the kitchen" …
  → Parsed: kitchen:1  confidence=87%
  → Navigating to kitchen:1 …
  · EXECUTING
  · RECOVERY_EXECUTING        ← geometric recovery fired
  · EXECUTING
  ✓ SUCCESS — reached kitchen:1
```

### 8.2 Preemption — typing a new command mid-navigation

You can type a new command at **any time** while the robot is navigating. The terminal immediately:

1. Fires `/cancel_navigation` → orchestrator cancels the active Nav2 goal.
2. Waits for the current goal to abort.
3. Starts the new goal without returning to the prompt.

```
[nav] > chair:2
  → Navigating to chair:2 …
  · EXECUTING

[nav] > living room      ← typed while navigating

⚡ Preempted → new command: 'living room'
  → Parsing NL: "living room" …
  → Parsed: living_room:1  confidence=91%
  → Navigating to living_room:1 …
```

### 8.3 Operator prompts (inline)

When the BT's `OperatorPrompt` node fires (e.g. for `open_door_then_replan` or `clear_object_then_replan` directives), the prompt appears inline in the same terminal:

```
  ╔══════════════════════════════════════════════════╗
  ║ OPERATOR PROMPT                                  ║
  ╚══════════════════════════════════════════════════╝
  Please open the door blocking the path to kitchen:1.
  Object : door:3
  Action : open_door_then_replan
  [operator] y / n > y
  ✓ Confirmed
```

Responding `y` acknowledges the directive and the BT continues. `n` rejects it.

### 8.4 Keyboard shortcuts

| Key | Effect |
|---|---|
| `Ctrl-C` | Cancel active navigation; return to prompt |
| `Ctrl-D` | Exit the terminal |
| New text + Enter | Preempt current navigation with new destination |

### 8.5 Headless / CI use

For unattended testing where operator prompts should be auto-acknowledged:

```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py enable_operator_io:=true operator_auto_ack_for_dev:=true
```

In this mode, `operator_io_node` serves `/operator_decision` and automatically confirms all prompts. Do **not** run `navigation_terminal` alongside `operator_io_node` — they both serve `/operator_decision` and will conflict.

---

## 9. Natural-language commands via the LLM navigator

`semantic_nav_llm` provides a `navigator_node` that converts a natural-language command (e.g. `"I am hungry"`) into a strictly structured intent using `llama_ros` and a GBNF grammar, then validates the target against `map_v001.json`. It **does not** publish motion commands or `PoseStamped` goals - it only provides a typed service response that the terminal can consume.

### 9.1 Start the llama_ros action server

The system launch starts `navigator_node` by default, but it does **not** start the model server. Start `llama_node` in a separate terminal using the `llama_ros` virtual environment:

```bash
source /opt/ros/humble/setup.bash
source ~/demo_bringup/install/setup.bash
source ~/demo_bringup/src/llama_ros/.venv/bin/activate

ros2 run llama_ros llama_node --ros-args \
  -r __ns:=/llama \
  --params-file ~/demo_bringup/llama_configs/llama_bringup.yaml
```

The YAML file should point to your local GGUF model, for example:

```yaml
/**:
  ros__parameters:
    model:
      path: "/home/shaker/demo_bringup/models/Meta-Llama-3-8B-Instruct-Q4_K_M.gguf"
      warmup: true

    context:
      n_ctx: 2048
      n_batch: 8
      n_predict: 128

    gpu:
      n_gpu_layers: 0

    cpu:
      n_threads: -1

    prompt:
      system_prompt_type: "Llama-3"
```

Verify the server is up:

```bash
ros2 action info /llama/generate_response
# Expected: Action servers: 1
```

> Important: do **not** launch Gazebo/system bringup from the `llama_ros` virtual environment. Use the normal ROS/system Python for Gazebo, RTAB-Map, Nav2, and all other nodes. Use the `llama_ros` virtual environment only in the terminal that runs `llama_node`.

### 9.2 Using NL commands through the terminal

Once `llama_node` is running and `navigator_node` is up (launched by `semantic_nav_system.launch.py` by default), simply type your command at the `[nav] >` prompt in the navigation terminal:

```
[nav] > I am hungry

  → Parsing NL: "I am hungry" …
  → Parsed: kitchen:1  confidence=88%
  → Navigating to kitchen:1 …
```

No separate orchestrator invocation is needed. The terminal calls `/parse_semantic_command` automatically when it detects free-form text that is not an object key.

To disable the LLM parser (baseline Nav2/RTAB-Map testing only):

```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py enable_llm:=false
```

In this mode the terminal still accepts direct object keys; NL commands will fail with a "parse service unavailable" error.

### 9.3 GBNF grammar

The grammar (`src/semantic_nav_llm/config/semantic_intent.gbnf`) constrains the LLM to emit exactly:

```json
{"action": "navigate|clarify|reject", "target": "string", "confidence": 0-100}
```

`target` is intentionally a free-form string — the navigator then **validates it against `map_v001.json`** (normalizing case, underscores, and stripping leading articles). This decouples language reasoning from world-state authority: the LLM cannot send the robot to a place that doesn't exist.

### 9.4 Navigator parameters

| Parameter | Default | Description |
|---|---|---|
| `service_name` | `/parse_semantic_command` | Service name this node provides |
| `llama_action` | `/llama/generate_response` | `llama_msgs/action/GenerateResponse` action to call |
| `grammar_path` | `<share>/semantic_nav_llm/config/semantic_intent.gbnf` | GBNF grammar enforced on LLM output |
| `llm_result_timeout_sec` | `180.0` | Timeout waiting for llama_ros inference result |
| `min_confidence_percent` | `60` | Reject `navigate` intents with confidence below this |
| `max_tokens` | `64` | Request-level generation cap for intent JSON |
| `debug_prompt` | `false` | Log the full prompt sent to the LLM |
| `debug_grammar` | `false` | Log the GBNF grammar attached to the request |

---

## 10. Semantic database

The system uses two complementary JSON databases, both loaded once at startup into an immutable `SemanticStore` snapshot.

### 10.1 Object-centric map — `map_v001.json`

The primary database. 118 detected object instances in the AWS Small House, each with a 3D bounding-box pose, semantic tag, LLM-generated caption, and navigability flag:

```json
{
  "objects": [
    {
      "key": "chair:2",
      "tag": "chair",
      "caption": "A wooden dining chair near the kitchen table",
      "navigable": true,
      "frame_id": "map",
      "x": 5.293, "y": 0.214, "z": 0.0,
      "bbox_x": 0.5, "bbox_y": 0.5,
      "mobility_state": "static"
    }
  ]
}
```

Resolution priority in `resolver_node`:
1. **Direct key lookup** (`target_object_key` field) — bypasses ranking entirely; used by the orchestrator's `exclude_object_key` recovery logic.
2. **Object tag + intent hint** — BM25/LLM/Hybrid caption-ranked candidates filtered by `navigable=true`; multiple instances of the same tag (e.g. 6 chairs) are ranked by score.
3. **Legacy room alias** — fallback to `semantic_db.json` if no object match.

The resolver uses `StandoffPlanner` to project the goal to a robot-reachable approach pose in front of the object (safe standoff distance, accounting for bounding-box extent).

### 10.2 Room-level database — `semantic_db.json`

Legacy fallback with 13 named room waypoints in the map frame. Used when a query matches no object tag.

```json
{
  "locations": {
    "kitchen": { "frame_id": "map", "x": 13.2217, "y": -0.2792, "yaw": 0.0, "aliases": ["kitchen"] }
  }
}
```

### 10.3 Action attribute sidecar — `object_action_attributes.json`

Per-tag flags consumed by the BT-led recovery directive builder:

```json
{ "tags": { "chair": { "openable": false, "clearable": true, "safety_class": "none" }, "door": { "openable": true, "clearable": false, "safety_class": "semi-static" } } }
```

- `clearable` → recovery can issue `wait_then_replan` (object may move)
- `openable` → human intervention may clear the path
- `safety_class: "human"` or `"animal"` → triggers `signal_wait_recheck` instead of `give_up`

### 10.4 Versioning

The resolver builds an immutable `SemanticStore` snapshot at startup and stamps every `ResolveLocation` response with `db_version` and `db_stamp` (file mtime). The orchestrator propagates these through resolution → execution log lines so any run can be traced to a specific DB state.

### 10.5 Coordinates are world-specific

All coordinates are measured for the **AWS Small House**. If you swap worlds, re-measure object poses via teleop + RViz and update `map_v001.json`.

---

## 11. Launch arguments reference

### `semantic_nav_system.launch.py`

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use the Gazebo simulation clock |
| `localization` | `false` | Run RTAB-Map in localization mode instead of SLAM |
| `rviz` | `true` | Launch RViz (with `respawn=true`; a RViz crash no longer kills the stack) |
| `x_pose` | `0.0` | Initial TurtleBot3 X position in Gazebo |
| `y_pose` | `0.0` | Initial TurtleBot3 Y position in Gazebo |
| `aws_small_house_path` | (absolute path) | Path to the AWS world source dir (must contain `worlds/small_house.world`) |
| `nav2_params_file` | `<share>/semantic_nav_bringup/config/nav2_semantic_params.yaml` | Nav2 params file |
| `enable_llm` | `true` | Launch `semantic_nav_llm`'s `navigator_node` |
| `semantic_map_path` | `<share>/semantic_nav_semantics/config/map_v001.json` | Object-centric semantic map for resolver and LLM nodes |
| `llama_action` | `/llama/generate_response` | `llama_ros` action endpoint consumed by `navigator_node` |
| `parse_service` | `/parse_semantic_command` | Service name exposed by `navigator_node` |
| `propose_recovery_service` | `/propose_recovery` | Recovery proposal service exposed by `navigator_node` |
| `grammar_path` | `<share>/semantic_nav_llm/config/semantic_intent.gbnf` | GBNF grammar for LLM intent output |
| `recovery_grammar_path` | `<share>/semantic_nav_llm/config/recovery_intent.gbnf` | GBNF grammar for recovery proposals |
| `min_confidence_percent` | `60` | Minimum LLM confidence for accepted navigate intents |
| `max_tokens` | `64` | Token cap for intent JSON generation |
| `recovery_max_tokens` | `256` | Token cap for recovery JSON generation |
| `llm_result_timeout_sec` | `180.0` | Timeout waiting for llama_ros inference result |
| `debug_prompt` | `false` | Print prompt sent from `navigator_node` to `llama_ros` |
| `debug_grammar` | `false` | Print GBNF grammar attached to the request |
| `enable_operator_io` | `false` | Launch `operator_io_node`. Default `false` — `navigation_terminal` handles `/operator_decision` inline |
| `operator_auto_ack_for_dev` | `false` | Auto-acknowledge all operator prompts (CI/headless only) |
| `operator_prompt_timeout_sec` | `0.0` | Stdin timeout for `operator_io_node`; 0 = no timeout |

**If you cloned the AWS package elsewhere**, pass `aws_small_house_path:=/your/path` on the launch command line or edit the default in `semantic_nav_system.launch.py`.

To launch without the LLM parser (baseline testing):
```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py enable_llm:=false
```

---

## 12. ROS 2 endpoints

### Core pipeline

| Endpoint | Type | Provider |
|---|---|---|
| `/resolve_location` | Service (`ResolveLocation`) | `semantic_nav_semantics/resolver_node` |
| `/validate_pose_goal` | Service (`ValidatePose`) | `semantic_nav_validator/validator_node` |
| `/execute_pose` | Action (`ExecutePose`) | `semantic_nav_executor/executor_node` |
| `/parse_semantic_command` | Service (`ParseSemanticCommand`) | `semantic_nav_llm/navigator_node` (optional) |
| `/llama/generate_response` | Action (`llama_msgs/action/GenerateResponse`) | external (`llama_ros`), consumed by `semantic_nav_llm` |

### Terminal ↔ orchestrator

| Endpoint | Type | Provider / Consumer |
|---|---|---|
| `/navigate_to_query` | Service (`NavigateToQuery`) | served by `navigation_orchestrator`; called by `navigation_terminal` |
| `/cancel_navigation` | Service (`std_srvs/Trigger`) | served by `navigation_orchestrator`; called by `navigation_terminal` on preemption |
| `/operator_decision` | Service (`OperatorDecision`) | served by `navigation_terminal` (default) or `operator_io_node` (`enable_operator_io:=true`) |
| `/recovery_status` | Topic (`std_msgs/String`) | published by `navigation_orchestrator`; subscribed by `navigation_terminal` for live status display |

### BT-led recovery

| Endpoint | Type | Provider / Consumer |
|---|---|---|
| `/request_recovery` | Service (`RequestRecovery`) | served by `navigation_orchestrator`; called by `EscalateToLLMRecovery` C++ BT node |
| `/propose_recovery` | Service (`ProposeRecovery`) | served by `navigator_node` (LLM); called by `navigation_orchestrator` |
| `/match_responsible_object` | Service (`MatchResponsibleObject`) | served by `navigation_orchestrator`; called by `QuerySemanticContext` C++ BT node |
| `/refresh_local_objects` | Service (`RefreshLocalObjects`) | served by `local_object_query_node`; called by `QuerySemanticContext` C++ BT node |
| `/robot_obstacle_signal` | Topic (`std_msgs/String`) | published by `EmitObstacleSignal` BT node for `signal_wait_recheck` scenarios |

### Direct service calls (debugging)

```bash
# Resolve an object tag
ros2 service call /resolve_location semantic_nav_interfaces/srv/ResolveLocation \
  "{object_tag: 'chair', intent_hint: ''}"

# Direct key lookup (bypass ranking)
ros2 service call /resolve_location semantic_nav_interfaces/srv/ResolveLocation \
  "{target_object_key: 'chair:2'}"

# Validate a pose
ros2 service call /validate_pose_goal semantic_nav_interfaces/srv/ValidatePose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: '', use_start: false}"

# Trigger navigation programmatically (bypasses terminal)
ros2 service call /navigate_to_query semantic_nav_interfaces/srv/NavigateToQuery \
  "{query: 'chair:2', nl_command: ''}"

# Cancel active navigation
ros2 service call /cancel_navigation std_srvs/srv/Trigger "{}"
```

---

## 13. Interface definitions

**`ResolveLocation.srv`**
```
string query              # legacy room-name lookup (fallback)
string object_tag         # preferred: object tag to rank (e.g. "chair")
string intent_hint        # hint passed to caption ranker (e.g. "near the window")
string target_object_key  # direct key bypass (e.g. "chair:2"), skips ranking
---
bool success
string location_id        # resolved key (e.g. "chair:2" or "kitchen")
string object_key         # same as location_id for object-centric results
geometry_msgs/PoseStamped pose
uint32 db_version
builtin_interfaces/Time db_stamp
string message
```

**`ValidatePose.srv`**
```
geometry_msgs/PoseStamped goal
string planner_id
bool use_start
geometry_msgs/PoseStamped start
---
bool valid
string message
float32 path_length
uint32 pose_count
```

**`ParseSemanticCommand.srv`**
```
string command
---
bool success
string intent                # navigate_to_object | clarify | reject
string object_tag            # resolved object tag (e.g. "chair")
string intent_hint           # spatial qualifier from LLM (e.g. "near the window")
string target_object_key     # direct key if LLM resolves to a specific instance
bool target_known
uint8 confidence_percent
string raw_output
string message
```

**`NavigateToQuery.srv`**
```
string query          # object key (e.g. "chair:2") or NL-resolved tag/key
string nl_command     # original NL text; empty for direct commands
---
bool success
string outcome        # REACHED | RESOLUTION_FAILED | EXECUTION_FAILED | BUSY | INVALID
string failure_reason
```

**`OperatorDecision.srv`**
```
string prompt_text
string responsible_object_key
string failure_stage
string directive_action
string recovery_event_id
---
bool acknowledged
string operator_note
```

**`ExecutePose.action`**
```
# Goal
geometry_msgs/PoseStamped pose
string object_key
string behavior_tree
uint32 db_version
builtin_interfaces/Time db_stamp
---
# Result
bool success
string message
---
# Feedback
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
```

**`RequestRecovery.srv`** (BT → orchestrator)
```
string trigger_source          # "bt_recovery_plugin"
string failure_stage           # "execution"
string nav2_message
string original_object_tag
string original_intent_hint
string current_target_object_key
string responsible_object_key
string responsible_object_tag
string robot_pose_summary
float32 distance_remaining_at_abort
int32 remaining_retry_budget
---
bool success
string status                  # "directive_issued" | "terminal_fail"
string action                  # "retry_target" | "wait_then_replan" | "give_up" | ""
geometry_msgs/PoseStamped new_goal_pose
string new_object_key
int32 wait_seconds
string signal_class
int32 signal_attempts
string recovery_event_id
int32 attempts_used
int32 retry_cap
string message
```

**`ProposeRecovery.srv`** (orchestrator → LLM navigator)
```
string original_target
string original_object_tag
string failure_stage
string trigger_source
string nav2_message
string robot_pose_summary
string responsible_object_key
string responsible_object_tag
int32 remaining_retry_budget
---
bool success
string action                  # "retry_target" | "wait_then_replan" | "give_up"
string target_object_tag
string target_intent_hint
int32 wait_seconds
string signal_class
int32 signal_attempts
int32 confidence
string message
```

---

## 14. Troubleshooting

### Gazebo shows an empty house, log spams `Error Code 12 Msg: Unable to find uri[model://aws_robomaker_residential_*]`
`GAZEBO_MODEL_PATH` does not include the AWS models directory. See [Section 6](#6-environment-variables-critical). The most common cause is a `~/.bashrc` line that uses `=` instead of `:` to set the path, clobbering it on shell startup.

### `[Err] [RenderEngine.cc:197] Failed to initialize scene` followed by a null-`Scene`/`Camera` boost assert
A previous `gzserver`/`gzclient` is still alive and holding the X display. Kill them and re-run:
```bash
pkill -9 gzserver; pkill -9 gzclient
```
If the error persists on a fresh boot, suspect a GPU/Ogre problem. Confirm with `glxinfo -B` (install `mesa-utils` if missing) and try `LIBGL_ALWAYS_SOFTWARE=1` as a sanity check.

### `Package 'rtabmap_demos' not found` (or `aws_robomaker_small_house_world`)
You forgot to source one of the workspaces. Re-do the `source` commands in [Section 7](#7-running-the-system), in order. If `rtabmap_demos` is missing entirely, you have not built `demo_bringup` — see [Section 5](#5-building).

### `LLM action server '/llama/generate_response' not available after 30.0s`
The `llama_ros` action server is not running, or it advertises under a different name. Confirm with `ros2 action list`, and pass the correct name via `--ros-args -p llama_action:=/your/action/name` when starting `navigator_node`.

### `Invalid strict JSON from LLM: …`
The LLM produced non-JSON output even though the GBNF grammar was attached. Verify with `ros2 interface show llama_msgs/msg/SamplingConfig` that the field `grammar` exists; if not, update `llama_ros` to a version that supports GBNF. As a temporary debugging aid, set `allow_json_extraction_fallback:=true` — never leave this on in production.

### `LLM target='…' is not known in map_v001.json`
The LLM resolved to a label your DB does not contain. Either add the object to `map_v001.json` (and restart `resolver_node`) or rephrase the command.

### Validation fails for a known object ("chair:2")
The robot has not yet explored the area containing that object. Drive the robot with teleop until the room is visible in RViz's occupancy grid, then retry. This is by design — see Section 15, principle 6.

### Robot spawns inside a wall
The default `x_pose=0.0 y_pose=0.0` may not be in free space depending on AWS house origin. Pass spawn coordinates that you have visually confirmed are clear:
```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py x_pose:=-2.0 y_pose:=0.5
```

### `/operator_decision` service conflict
Both `navigation_terminal` and `operator_io_node` serve `/operator_decision`. Running both simultaneously causes one to fail. Use one or the other: `navigation_terminal` for interactive use (default); `operator_io_node` with `enable_operator_io:=true` and `operator_auto_ack_for_dev:=true` for headless/CI runs. Do not start `navigation_terminal` in CI.

### RViz crashes mid-session
RViz is launched with `respawn=true` in the bringup and will restart automatically after 2 seconds. The rest of the stack (Nav2, RTAB-Map, semantic nodes) continues running uninterrupted.

---

## 15. Design principles

1. **Modular ROS 2 package separation** — each concern in its own package.
2. **No direct LLM-to-motion coupling** — the LLM emits a structured `{action, object_tag, intent_hint, confidence}` intent only. Targets are validated against the semantic DB; the LLM cannot fabricate coordinates, motion commands, behavior trees, or unknown object types.
3. **Validation before execution** — `ValidateSemantic` inside the BT performs a geometric veto before motion starts.
4. **Stage-specific logging and failure isolation** — `[LLM_INTENT]` / `[RESOLUTION]` / `[VALIDATION]` / `[EXECUTION]` / `[RECOVERY/BT]` prefixes.
5. **Live RTAB-Map map** — Nav2 consumes the live `/map` topic, no pre-saved map file.
6. **Semantic navigation only for explored regions** — goals must be in mapped free space at the moment the query is issued.
7. **Versioned semantic snapshots** — every resolution stamps its response with `db_version` and `db_stamp`, and the orchestrator propagates them through execution so any single run can be traced back to a specific DB state.
8. **Strict grammar-constrained LLM output** — a GBNF grammar enforces the JSON schema at decode time; a confidence floor and a DB membership check gate acceptance. A separate `recovery_intent.gbnf` constrains recovery proposals.
9. **Nav2 BT owns the full recovery flow** — three-tier strategy entirely inside `semantic_recovery_bt.xml`. The orchestrator dispatches a single `ExecutePose` goal and only responds to explicit `/request_recovery` calls from the BT plugin (`EscalateToLLMRecovery`). `PathClearCondition` is not in the primary navigation sequence — `RateController(1 Hz)` around `ComputePathToPose` handles replanning around small navigable obstacles without triggering recovery. `PathClearCondition` is kept only inside the `SignalWaitRecheck` subtree for per-attempt clear-path confirmation during animate-blocker recovery.
10. **Object-instance multi-disambiguation via `exclude_object_key`** — when a `retry_target` directive would resolve back to the already-blocked instance, `_resolve_target_for_directive` walks all same-tag instances and selects the nearest non-blocked one via `target_object_key` direct lookup (Precedence 1 in the resolver, bypassing caption ranking).
11. **Retry budget is BT-authoritative** — the Nav2 BT's `RecoveryNode(number_of_retries=3)` controls the outer retry count; the orchestrator tracks `remaining_retry_budget` separately and returns a `give_up` directive when it reaches 0.
12. **Object-centric resolution with ranked candidates** — the resolver scores all instances of an object tag using BM25/LLM/Hybrid caption ranking and returns the highest-scoring navigable instance. Offline eval on 40 single-tag fixtures (GPU-accelerated llama_ros):

    | Ranker | Top-1 | Top-3 | LLM invocation rate | Notes |
    |---|---|---|---|---|
    | BM25 | 87.5% | 100.0% | 0% | Baseline; zero LLM cost |
    | BM25+spatial | 90.0% | 97.5% | 0% | |
    | LLM-text | 90.0% | 95.0% | 100% | ~3.2 s/query |
    | LLM+spatial | 87.5% | 95.0% | 100% | |
    | **Hybrid δ=0.5** | **90.0%** | **100.0%** | **48%** | **Production default** |

    Hybrid δ=0.5 matches pure-LLM top-1 accuracy, achieves 100% top-3, and invokes the LLM on only 48% of queries — halving average latency. Production default is `ranker=bm25`; switch to `ranker=hybrid` with llama_ros running to activate the full hybrid path.

### Expected behavior summary

| Scenario | Behavior |
|---|---|
| Unknown query | Resolution fails, pipeline stops |
| LLM rejects command (e.g. `"drive forward"`) | NavigatorNode returns `intent=reject`; orchestrator not invoked |
| LLM proposes object tag not in DB | NavigatorNode returns `success=false`; orchestrator not invoked |
| LLM confidence below `min_confidence_percent` | NavigatorNode returns `success=false`; orchestrator not invoked |
| Known object, unexplored area | `ValidateSemantic` in BT rejects before motion starts |
| Small navigable obstacle mid-path | `RateController(1 Hz)` triggers replanning; FollowPath follows updated plan; no recovery fires |
| Costmap artifact / transient stuck | Geometric recovery (Tier 2): clear costmaps + BackUp; primary retried |
| Full path blockage | Geometric recovery fails; Tier 3 semantic recovery: `CaptureBlockageContext` → `QuerySemanticContext` → `EscalateToLLMRecovery` → directive |
| Blocked object has same-tag alternatives | `exclude_object_key` redirects `retry_target` to nearest non-blocked instance |
| Retry budget exhausted (3 BT attempts) | Orchestrator returns `give_up`; BT `GiveUpTerminal` executes; `ExecutePose` returns failure |
| Human/animal blocking path | `signal_wait_recheck`: robot emits polite-clear signal, waits, re-checks via `PathClearCondition` |
| Clearable non-animate object blocking | `clear_object_then_replan`: operator prompted inline in terminal to remove obstacle |
| Door blocking path (`responsible_openable=True`) | `open_door_then_replan`: operator prompted inline in terminal to open door |
| User types new command mid-navigation | Terminal fires `/cancel_navigation`; orchestrator aborts Nav2 goal; new goal started immediately |

---

## 16. Roadmap

1. **Live semantic DB topic ingestion** — re-add a `SemanticDB` topic subscriber to `resolver_node` that atomically replaces the snapshot at runtime, bumping `db_version`. The interfaces (`SemanticDB.msg`, `db_version`/`db_stamp` fields on `ResolveLocation`) and the orchestrator-side propagation are already in place; only the subscriber needs to come back.
2. ~~**Recovery Orchestration with LLM**~~ — **DONE (BT-LR M1–M4, 2026-06-12).** Nav2 BT with `PathClearCondition`, `QuerySemanticContext`, `EscalateToLLMRecovery`, `EmitObstacleSignal`, and `ValidateSemantic` C++ plugins; GBNF-constrained `ProposeRecovery` endpoint; `exclude_object_key` multi-instance disambiguation; up to 3 retries with `give_up` terminal.
3. ~~**E2E Scenario 2 & 3 validation**~~ — **DONE (BT-LR M5, 2026-06-18).** `OperatorPrompt` BT node + `operator_io_node` + `open_door_then_replan` and `clear_object_then_replan` directives validated. Scenario 2a: clearable obstacle → `clear_object_then_replan`. Scenario 2b: non-clearable → `wait_then_replan`. Scenario 3: human-class → `signal_wait_recheck`.
4. ~~**Three-tier BT recovery + navigation terminal**~~ — **DONE (BT-LR M6, 2026-06-20).** `PathClearCondition` removed from primary navigation path. `RateController(1 Hz)` around `ComputePathToPose` handles small navigable obstacles. `RoundRobin` recovery child: Tier 2 geometric (clear + backup), Tier 3 semantic (`CaptureBlockageContext` → `QuerySemanticContext` → `EscalateToLLMRecovery`). Unified `navigation_terminal` replaces one-shot CLI and `operator_io_node`; supports NL commands, direct object keys, mid-navigation preemption, and inline operator prompts. `sample_radius_m=0.0` bug fixed in `PathClearCondition`; severity gating added (`BlockageMetrics`, `min_blocked_samples`, `min_blocked_length_m`, `blocked_fraction_threshold`).
5. **Optional llama_ros launch integration** — `navigator_node` is now launched by `semantic_nav_system.launch.py` by default, but the heavyweight `llama_node` model server is still launched separately. A future launch file may optionally include `llama_node` once model startup, Python environment isolation, and memory behavior are stable.
6. ~~**Ranker evaluation with live LLM**~~ — **DONE (2026-06-18).** Full 40-fixture offline eval with GPU-accelerated llama_ros. Results in `eval/results_full.csv`; plots in `eval/`. Production recommendation: `hybrid δ=0.5` (90% top-1, 100% top-3, 48% LLM invocation rate). See §15.12 for full table.
7. **Post-failure orchestrator policies** — re-resolve / re-validate on failure, optionally gated on `db_version` change.
8. **Snapshot immutability during active navigation** — when live DB updates return, ensure an in-flight goal keeps using the snapshot it was resolved against.
