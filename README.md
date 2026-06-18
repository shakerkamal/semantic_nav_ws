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
8. [Sending a navigation query](#8-sending-a-navigation-query)
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
2. **Resolves** the object tag to a `PoseStamped` in the `map` frame using an object-centric semantic database (`map_v001.json`, 117 objects), ranked by BM25/LLM/Hybrid caption scoring, and stamps the response with `db_version`/`db_stamp`.
3. **Validates** that Nav2's planner can currently compute a path to that pose (catches goals in unmapped or blocked areas before committing).
4. **Executes** the navigation via Nav2's `NavigateToPose`, using a custom Nav2 behavior tree (`semantic_recovery_bt.xml`) that monitors for path blockages and autonomously invokes LLM-driven recovery.
5. **Recovers** (if blocked): the Nav2 BT detects a path obstruction via `PathClearCondition`, queries semantic context via `QuerySemanticContext`, escalates to the LLM via `EscalateToLLMRecovery` → `/request_recovery`, and the orchestrator issues a directive (`retry_target`, `wait_then_replan`, or `give_up`) with up to 3 retries, excluding previously-blocked object instances when selecting alternatives.

Each stage gates the next: if parsing/resolution fails, downstream stages are skipped; if validation fails, execution is skipped. RTAB-Map provides live SLAM (no pre-saved map), so semantic destinations must already be in the explored, navigable free space at the time you issue the query.

---

## 2. System architecture

### Pipeline

```
User intent ("I am hungry")
    │
    ▼
NavigatorNode (LLM + GBNF) -> {action: navigate, object_tag: "chair", intent_hint: "...", confidence: 92}
    │  (or a direct semantic query, skipping the LLM step)
    ▼
ResolveLocation  (object_tag -> BM25/Hybrid-ranked ObjectRow -> PoseStamped + db_version/db_stamp)
    │
    ▼
ValidatePose    (Nav2 ComputePathToPose feasibility check)       [skipped in bt_led mode]
    │
    ▼
ExecutePose     (Nav2 NavigateToPose with semantic_recovery_bt.xml)
    │                │
    │          ┌─────▼──────────────────────────────────────────────┐
    │          │  BT-Led Recovery (on path blockage)                 │
    │          │  PathClearCondition → QuerySemanticContext           │
    │          │  → EscalateToLLMRecovery → /request_recovery        │
    │          │  → orchestrator: retry_target | wait_then_replan    │
    │          │                  | signal_wait_recheck | give_up    │
    │          └─────────────────────────────────────────────────────┘
    ▼
Robot moves (or graceful give_up after 3 retries)
```

### Package layout (this workspace, `src/`)

| Package | Type | Responsibility |
|---|---|---|
| `semantic_nav_interfaces` | ament_cmake | Custom msgs/srvs/actions (`ResolveLocation`, `ValidatePose`, `ExecutePose`, `RequestRecovery`, `ProposeRecovery`, `MatchResponsibleObject`, `RefreshLocalObjects`, `ObjectInstance`, `RecoveryTrigger`) |
| `semantic_nav_semantics`  | ament_python | Object-centric semantic resolution: `SemanticStore` (immutable `map_v001.json` snapshot), `resolver_node`, `local_object_query_node`, `StandoffPlanner`, BM25/LLM/Hybrid caption rankers, `SpatialContextBuilder` |
| `semantic_nav_validator`  | ament_python | Path-existence check via `ComputePathToPose` |
| `semantic_nav_executor`   | ament_python | Bridges custom `ExecutePose` action to Nav2 `NavigateToPose` |
| `semantic_nav_orchestrator` | ament_python | Pipeline controller (standalone and `bt_led` modes); serves `/request_recovery` with LLM-backed directive generation, `exclude_object_key` disambiguation, and 3-retry cap |
| `semantic_nav_llm`        | ament_python | NL → constrained semantic intent (llama_ros + GBNF); serves `/propose_recovery` for BT-led LLM recovery proposals |
| `semantic_nav_nav2_plugins` | ament_cmake (C++) | Nav2 BT plugins: `PathClearCondition`, `QuerySemanticContext`, `EscalateToLLMRecovery`, `EmitObstacleSignal`, `ValidateSemantic`; BT XML: `semantic_recovery_bt.xml` |
| `semantic_nav_path_monitor` | ament_python | `plan_intersection_monitor`: monitors Nav2 plan for intersections with semantic obstacles in the costmap |
| `semantic_nav_bringup`    | ament_python | Launch files for the integrated stack |

---

## 3. Required workspaces and packages

This project lives across **two workspaces** that both need to be sourced at runtime:

| Workspace | Holds | Built? |
|---|---|---|
| `~/demo_bringup` | RTAB-Map ROS 2 sources (`rtabmap_ros`, `rtabmap`) and `llama_ros` sources inside `src/`; `aws-robomaker-small-house-world/` cloned as a sibling of `src/` | RTAB-Map and `llama_ros` yes (colcon). AWS world **no** - we only consume its `worlds/` and `models/` assets |
| `~/semantic_nav_ws` | **This repo** - all `semantic_nav_*` packages | Yes |

> If you do not have `~/demo_bringup` yet, follow [Section 4](#4-first-time-setup-step-by-step) below to create it.

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

### 4.2 Set up `~/demo_bringup` (RTAB-Map + llama_ros sources + AWS world assets)

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

Build the two workspaces **in this order** (`~/demo_bringup` first, then this repo):

```bash
# 1. ~/demo_bringup builds both rtabmap_* and llama_ros from src/
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

The `aws-robomaker-small-house-world/` directory at `~/demo_bringup/` is intentionally outside `src/`, so colcon will ignore it. If any build fails with "package not found", you almost certainly forgot to source `~/demo_bringup/install/setup.bash` before building this workspace.

---

## 6. Environment variables (critical)

Four variables can be set before any launch, in **every terminal** you use:

```bash
export TURTLEBOT3_MODEL=waffle
export TURTLEBOT3_GAZEBO_MODELS=/opt/ros/humble/share/turtlebot3_gazebo/models
export AWS_SMALL_HOUSE=$HOME/demo_bringup/aws-robomaker-small-house-world
export GAZEBO_MODEL_PATH=$AWS_SMALL_HOUSE/models:$TURTLEBOT3_GAZEBO_MODELS
```

**OR** update the `.bashrc` file with the following commands:

```bash
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
echo 'export TURTLEBOT3_GAZEBO_MODELS=/opt/ros/humble/share/turtlebot3_gazebo/models' >> ~/.bashrc
echo 'export AWS_SMALL_HOUSE=$HOME/demo_bringup/aws-robomaker-small-house-world' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$AWS_SMALL_HOUSE/models:$TURTLEBOT3_GAZEBO_MODELS' >> ~/.bashrc

source ~/.bashrc
```


**Why `GAZEBO_MODEL_PATH` matters:** the AWS world references its 60+ furniture meshes via `model://aws_robomaker_residential_*` URIs. Gazebo resolves those URIs by scanning `GAZEBO_MODEL_PATH`. The AWS world is asset-only (not built), so nothing exports `GAZEBO_MODEL_PATH` for you - you must set it manually or the Gazebo window opens empty and you get dozens of `Error Code 12 Msg: Unable to find uri[model://...]` errors.

**Common pitfall:** if your `.bashrc` already has a line like `export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models` (using `=`, not appending), it overwrites the variable every time a new shell starts and clobbers any AWS path you add. Replace any such line with the colon-joined version above, then open a fresh terminal and verify:

```bash
echo $GAZEBO_MODEL_PATH
# Expected: both paths joined with a colon
```

For convenience, add the following block to the **end** of `~/.bashrc`:

```bash
# --- semantic_nav: env ---
export TURTLEBOT3_MODEL=waffle
export TURTLEBOT3_GAZEBO_MODELS=/opt/ros/humble/share/turtlebot3_gazebo/models
export AWS_SMALL_HOUSE=$HOME/demo_bringup/aws-robomaker-small-house-world
export GAZEBO_MODEL_PATH=$AWS_SMALL_HOUSE/models:$TURTLEBOT3_GAZEBO_MODELS
# --- end semantic_nav ---
```

---

## 7. Running the system

Open a new terminal, source the two workspaces in this order, then launch:

```bash
source /opt/ros/humble/setup.bash
source ~/demo_bringup/install/setup.bash
source ~/semantic_nav_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle
export TURTLEBOT3_GAZEBO_MODELS=/opt/ros/humble/share/turtlebot3_gazebo/models
export AWS_SMALL_HOUSE=$HOME/demo_bringup/aws-robomaker-small-house-world
export GAZEBO_MODEL_PATH=$AWS_SMALL_HOUSE/models:$TURTLEBOT3_GAZEBO_MODELS

ros2 launch semantic_nav_bringup semantic_nav_system.launch.py
```

Sourcing `~/demo_bringup/install/setup.bash` makes RTAB-Map and the `llama_ros` message/action interfaces available. The `llama_ros` model server itself is still started separately, because model loading is heavyweight and may use a separate Python virtual environment.

### What happens

The launch starts components in a staggered order to avoid race conditions:

| t (s) | What starts |
|---|---|
| 0 | Gazebo (gzserver + gzclient) loads the AWS small house, robot_state_publisher, spawns TurtleBot3 Waffle |
| 0 | `semantic_resolver`, `semantic_nav_validator`, `semantic_nav_executor` nodes |
| 0 | `navigator_node` from `semantic_nav_llm` by default (`enable_llm:=true`) |
| 3 | RTAB-Map (`turtlebot3_rgbd_scan.launch.py`) |
| 5 | Nav2 (`navigation_launch.py`) and RViz |

You should see Gazebo render the textured house, then RViz appear with a growing occupancy grid as RTAB-Map builds the map.

The LLM parser service is launched by default:

```bash
ros2 service list | grep parse_semantic
# /parse_semantic_command
```

To disable the LLM parser for baseline Gazebo/Nav2/RTAB-Map debugging:

```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py enable_llm:=false
```

> Important: do **not** launch Gazebo/system bringup from the `llama_ros` virtual environment. Use the normal ROS/system Python for Gazebo, RTAB-Map, Nav2, and `spawn_entity.py`. Use the `llama_ros` virtual environment only in terminals that run `llama_node` or other `llama_ros` tooling.

### Driving the robot manually (to explore the map)

Open a **second** terminal (with `/opt/ros/humble/setup.bash` sourced) and run:

```bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

Drive the robot through rooms until the area containing your target location is visible in RViz's occupancy grid. Semantic navigation will fail to validate goals in unexplored space.

## 8. Sending a navigation query

Open a **third** terminal, source the same scripts as in Section 7, then use either a direct semantic query or a natural-language command.

### 8.1 Direct semantic query

```bash
ros2 run semantic_nav_orchestrator navigation_orchestrator --ros-args -p query:=kitchen
```

Or use the positional shorthand:

```bash
ros2 run semantic_nav_orchestrator navigation_orchestrator kitchen
ros2 run semantic_nav_orchestrator navigation_orchestrator living room
```

This path bypasses the LLM completely:

```
query -> ResolveLocation -> ValidatePose -> ExecutePose
```

### 8.2 Natural-language command

Requires:

1. `llama_node` running separately and exposing `/llama/generate_response`.
2. `navigator_node` running, usually through `semantic_nav_system.launch.py` because `enable_llm` defaults to `true`.

Then run:

```bash
ros2 run semantic_nav_orchestrator navigation_orchestrator --ros-args \
  -p command:="I am hungry"
```

This path is:

```
command -> ParseSemanticCommand -> ResolveLocation -> ValidatePose -> ExecutePose
```

If both `query` and `command` are provided, the orchestrator uses the direct `query` and bypasses LLM parsing.

The orchestrator logs each stage and includes the active semantic DB version where applicable:

```
[INTENT] Parsing natural-language command: 'I am hungry'
[INTENT] Parser response: success=True, intent='navigate_to_location', location_query='kitchen', canonical_location_id='kitchen', ...
[RESOLUTION] Resolved 'kitchen' -> location_id='kitchen', db_version=1, db_stamp=..., x=13.222, y=-0.279
[VALIDATION] Validation succeeded (location_id='kitchen', db_version=1): ..., path_length=..., pose_count=...
[EXECUTION] Sending goal to execute_pose action server (location_id='kitchen', db_version=1, ...)
[EXECUTION] Executor finished with status=SUCCEEDED(4), success=True, ...
```

If a stage fails, the prefix tells you exactly which one (`[INTENT]`, `[RESOLUTION]`, `[VALIDATION]`, `[EXECUTION]`).

### Orchestrator parameters

| Parameter | Default | Description |
|---|---|---|
| `query` | `''` | Direct semantic location name to navigate to. Positional CLI args are accepted as an override |
| `command` | `''` | Natural-language command to parse via `/parse_semantic_command` |
| `parse_service` | `/parse_semantic_command` | LLM parser service used when `command` is provided |
| `resolve_service` | `/resolve_location` | Resolution service name |
| `validate_service` | `/validate_pose_goal` | Validation service name |
| `execute_action` | `/execute_pose` | Execution action name |
| `planner_id` | `''` | Nav2 planner to use (empty = default) |
| `behavior_tree` | `''` | Behavior tree XML to forward to Nav2 (empty = default) |
| `enable_validation` | `true` | Skip validation stage if set to `false` |
| `service_wait_timeout_sec` | `10.0` | How long to wait for services to become available |
| `service_call_timeout_sec` | `10.0` | Per-call timeout for parser, resolver, and validator services |
| `action_server_wait_timeout_sec` | `10.0` | How long to wait for the execute_pose action server |
| `action_send_goal_timeout_sec` | `10.0` | Timeout for sending the goal to the action server |
| `execution_timeout_sec` | `300.0` | Overall execution timeout. Set ≤ 0 for no timeout |

## 9. Natural-language commands via the LLM navigator

`semantic_nav_llm` provides a `navigator_node` that converts a natural-language command (e.g. `"I am hungry"`) into a strictly structured intent using `llama_ros` and a GBNF grammar, then validates the target against your `semantic_db.json`. It **does not** publish motion commands or `PoseStamped` goals - it only provides a typed service response that the orchestrator can consume.

### 9.1 Start the llama_ros action server

The system launch starts `navigator_node` by default, but it does **not** start the model server. Start `llama_node` in a separate terminal.

Use the `llama_ros` virtual environment only in this terminal:

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
      path: "/home/shaker/Thesis/Implementation/demo_bringup/models/Meta-Llama-3-8B-Instruct-Q4_K_M.gguf"
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

If the action name differs, pass it through with the `llama_action` launch argument or node parameter.

### 9.2 Start the full system with the navigator enabled

In a separate terminal, do **not** activate the `llama_ros` virtual environment:

```bash
source /opt/ros/humble/setup.bash
source ~/demo_bringup/install/setup.bash
source ~/semantic_nav_ws/install/setup.bash

ros2 launch semantic_nav_bringup semantic_nav_system.launch.py
```

The system launch defaults to:

```bash
enable_llm:=true
```

Verify the parser service exists:

```bash
ros2 service list | grep parse_semantic
# /parse_semantic_command
```

For isolated parser debugging without the full Gazebo stack, you can still run:

```bash
ros2 run semantic_nav_llm navigator_node
```

### 9.3 Parse a natural-language command

```bash
ros2 service call /parse_semantic_command \
  semantic_nav_interfaces/srv/ParseSemanticCommand \
  "{command: 'I am hungry'}"
```

A successful parse returns:

```
success: true
intent: 'navigate_to_location'
location_query: 'kitchen'
canonical_location_id: 'kitchen'
confidence_percent: 92
location_known: true
raw_output: '{"action":"navigate","target":"kitchen","confidence":92}'
message: "Accepted navigation intent: target='kitchen', canonical_location_id='kitchen', confidence=92."
```

Other possible `intent` values:

| `intent` | Meaning |
|---|---|
| `navigate_to_location` | LLM proposed a navigation target and the canonical ID exists in `semantic_db.json` |
| `clarify` | LLM thinks the user wants navigation but the destination is ambiguous |
| `reject` | LLM determined the command is not a valid semantic navigation request (e.g. `"drive forward"`, `"turn left"`) |

A `reject` response is also returned when the LLM proposes a `navigate` action but the target is not present in the semantic DB or confidence is below the threshold - in this case `success=false` and the orchestrator should not proceed.

### 9.4 End-to-end: NL -> navigation

Once `llama_node`, `navigator_node`, Gazebo, RTAB-Map, Nav2, resolver, validator, and executor are running:

```bash
ros2 run semantic_nav_orchestrator navigation_orchestrator --ros-args \
  -p command:="I am hungry"
```

Expected flow:

```
[INTENT] command -> /parse_semantic_command -> canonical_location_id
[RESOLUTION] canonical_location_id -> PoseStamped + db_version/db_stamp
[VALIDATION] ComputePathToPose feasibility check
[EXECUTION] ExecutePose -> Nav2 NavigateToPose
```

### 9.5 Navigator parameters

| Parameter | Default | Description |
|---|---|---|
| `service_name` | `/parse_semantic_command` | Service name this node provides |
| `llama_action` | `/llama/generate_response` | `llama_msgs/action/GenerateResponse` action to call |
| `semantic_db_path` | `<share>/semantic_nav_semantics/config/semantic_db.json` | DB used to build the catalog of canonical names + aliases used for validation |
| `grammar_path` | `<share>/semantic_nav_llm/config/semantic_intent.gbnf` | GBNF grammar enforced on LLM output |
| `llama_wait_timeout_sec` | `30.0` | How long to wait for the llama action server |
| `llm_send_goal_timeout_sec` | `10.0` | Timeout for `send_goal_async` |
| `llm_result_timeout_sec` | `60.0` | Timeout for the LLM result future |
| `min_confidence_percent` | `60` | Reject `navigate` intents with confidence below this |
| `target_min_len` | `1` | Minimum accepted target length |
| `target_max_len` | `64` | Maximum accepted target length |
| `temperature` | `0.0` | LLM sampling temperature (kept at 0 for deterministic intent classification) |
| `top_k` | `1` | LLM top-k |
| `top_p` | `1.0` | LLM top-p |
| `max_tokens` | `64` | Request-level generation cap for intent JSON |
| `reset_context` | `true` | Reset the LLM context per call |
| `allow_json_extraction_fallback` | `false` | If GBNF is not enforced and the model emits prose, try to extract a JSON object (debug only - leave `false` in production) |
| `debug_prompt` | `false` | Log the full prompt sent to the LLM |
| `debug_grammar` | `false` | Log the GBNF grammar that is attached to the request |

### 9.6 GBNF grammar

The grammar (`src/semantic_nav_llm/config/semantic_intent.gbnf`) constrains the LLM to emit exactly:

```json
{"action": "navigate|clarify|reject", "target": "string", "confidence": 0-100}
```

`target` is intentionally a free-form string - the navigator then **validates it against `semantic_db.json`** (normalizing case, underscores, and stripping leading articles). This decouples language reasoning from world-state authority: the LLM cannot send the robot to a place that doesn't exist.

## 10. Semantic database

The system uses two complementary JSON databases, both loaded once at startup into an immutable `SemanticStore` snapshot.

### 10.1 Object-centric map — `map_v001.json`

The primary database. 117 detected object instances in the AWS Small House, each with a 3D bounding-box pose, semantic tag, LLM-generated caption, and navigability flag:

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

### 10.6 Overriding the DB at launch time

```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py \
  semantic_db_path:=/absolute/path/to/your_db.json
```

Restart `semantic_resolver` and `navigator_node` after any DB change.

---

## 11. Launch arguments reference

### `semantic_nav_system.launch.py`

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use the Gazebo simulation clock |
| `semantic_db_path` | `''` (use package default) | Absolute path to a `semantic_db.json` override for the semantic core/resolver |
| `semantic_db_topic` | `/semantic_nav/semantic_database` | Topic reserved for live semantic DB snapshots |
| `localization` | `false` | Run RTAB-Map in localization mode instead of SLAM |
| `rviz` | `true` | Launch RViz |
| `x_pose` | `0.0` | Initial TurtleBot3 X position in Gazebo |
| `y_pose` | `0.0` | Initial TurtleBot3 Y position in Gazebo |
| `aws_small_house_path` | (absolute path to the AWS world dir) | Path to the AWS world source dir (must contain `worlds/small_house.world`) |
| `enable_llm` | `true` | Launch `semantic_nav_llm`'s `navigator_node` |
| `llm_semantic_db_path` | `<share>/semantic_nav_semantics/config/semantic_db.json` | DB path used specifically by `navigator_node`; kept separate because the core `semantic_db_path` may intentionally be empty |
| `llama_action` | `/llama/generate_response` | `llama_ros` action endpoint consumed by `navigator_node` |
| `parse_service` | `/parse_semantic_command` | Service name exposed by `navigator_node` |
| `grammar_path` | `<share>/semantic_nav_llm/config/semantic_intent.gbnf` | GBNF grammar file for LLM output |
| `min_confidence_percent` | `60` | Minimum confidence for accepted navigate intents |
| `max_tokens` | `64` | Request-level generation cap for intent JSON |
| `llm_result_timeout_sec` | `60.0` | Timeout waiting for `llama_ros` inference result |
| `debug_prompt` | `false` | Print prompt sent from `navigator_node` to `llama_ros` |
| `debug_grammar` | `false` | Print GBNF grammar sent from `navigator_node` to `llama_ros` |

**If you cloned the AWS package elsewhere**, you must pass `aws_small_house_path:=/your/path` on the launch command line - or edit the default in `src/semantic_nav_bringup/launch/semantic_nav_system.launch.py`.

To launch the system without the LLM parser:

```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py enable_llm:=false
```

## 12. ROS 2 endpoints

### Core pipeline

| Endpoint | Type | Provider |
|---|---|---|
| `/resolve_location` | Service (`ResolveLocation`) | `semantic_nav_semantics/resolver_node` |
| `/validate_pose_goal` | Service (`ValidatePose`) | `semantic_nav_validator/validator_node` |
| `/execute_pose` | Action (`ExecutePose`) | `semantic_nav_executor/executor_node` |
| `/parse_semantic_command` | Service (`ParseSemanticCommand`) | `semantic_nav_llm/navigator_node` (optional) |
| `/llama/generate_response` | Action (`llama_msgs/action/GenerateResponse`) | external (`llama_ros`), consumed by `semantic_nav_llm` |

### BT-led recovery

| Endpoint | Type | Provider / Consumer |
|---|---|---|
| `/request_recovery` | Service (`RequestRecovery`) | served by `navigation_orchestrator`; called by `EscalateToLLMRecovery` C++ BT node |
| `/propose_recovery` | Service (`ProposeRecovery`) | served by `navigator_node` (LLM); called by `navigation_orchestrator` |
| `/match_responsible_object` | Service (`MatchResponsibleObject`) | served by `navigation_orchestrator`; called by `QuerySemanticContext` C++ BT node |
| `/refresh_local_objects` | Service (`RefreshLocalObjects`) | served by `local_object_query_node`; called by `QuerySemanticContext` C++ BT node |
| `/recovery_trigger` | Topic (`RecoveryTrigger`) | published by BT recovery plugins; subscribed by `navigation_orchestrator` |
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

# Query local objects near robot (radius in metres)
ros2 service call /refresh_local_objects semantic_nav_interfaces/srv/RefreshLocalObjects \
  "{robot_pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, radius_m: 3.0}"
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
string intent                # navigate_to_location | clarify | reject
string object_tag            # resolved object tag (e.g. "chair")
string intent_hint           # spatial qualifier from LLM (e.g. "near the window")
uint8 confidence_percent
string raw_output
string message
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
string robot_pose_summary      # nearest-neighbour text for LLM prompt
float32 distance_remaining_at_abort
int32 remaining_retry_budget
---
bool success
string status                  # "directive_issued" | "terminal_fail"
string action                  # "retry_target" | "wait_then_replan" | "signal_wait_recheck" | "give_up" | ""
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
string original_target         # original object key or tag
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
string action                  # "retry_target" | "wait_then_replan" | "signal_wait_recheck" | "give_up"
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
You forgot to source one of the workspaces. Re-do the `source` commands in [Section 7](#7-running-the-system), in order. If `rtabmap_demos` is missing entirely, you have not built `~/demo_bringup` - see [Section 5](#5-building).

### `LLM action server '/llama/generate_response' not available after 30.0s`
The `llama_ros` action server is not running, or it advertises under a different name. Confirm with `ros2 action list`, and pass the correct name via `--ros-args -p llama_action:=/your/action/name` when starting `navigator_node`.

### `Invalid strict JSON from LLM: …`
The LLM produced non-JSON output even though the GBNF grammar was attached. Verify with `ros2 interface show llama_msgs/msg/SamplingConfig` that the field `grammar` exists; if not, update `llama_ros` to a version that supports GBNF. As a temporary debugging aid, set `allow_json_extraction_fallback:=true` - never leave this on in production.

### `LLM target='…' is not known in semantic_db.json`
The LLM resolved to a label your DB does not contain. Either add an alias for that label in `semantic_db.json` (and restart `navigator_node`) or rephrase the prompt.

### Validation fails for a known location ("kitchen")
The robot has not yet explored the area containing that location. Drive the robot with teleop until the room is visible in RViz's occupancy grid, then retry the query. This is by design - see Section 15, principle 6.

### Robot spawns inside a wall
The default `x_pose=0.0 y_pose=0.0` may not be in free space depending on AWS house origin. Pass spawn coordinates that you have visually confirmed are clear:
```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py x_pose:=-2.0 y_pose:=0.5
```

---

## 15. Design principles

1. **Modular ROS 2 package separation** - each concern in its own package.
2. **No direct LLM-to-motion coupling** - the LLM emits a structured `{action, object_tag, intent_hint, confidence}` intent only. Targets are validated against the semantic DB; the LLM cannot fabricate coordinates, motion commands, behavior trees, or unknown object types.
3. **Validation before execution** - planner-based reachability check before committing to navigation (skipped in `bt_led` mode where `ValidateSemantic` inside the BT handles this).
4. **Stage-specific logging and failure isolation** - `[LLM_INTENT]` / `[RESOLUTION]` / `[VALIDATION]` / `[EXECUTION]` / `[RECOVERY/BT]` prefixes.
5. **Live RTAB-Map map** - Nav2 consumes the live `/map` topic, no pre-saved map file.
6. **Semantic navigation only for explored regions** - goals must be in mapped free space at the moment the query is issued.
7. **Versioned semantic snapshots** - every resolution stamps its response with `db_version` and `db_stamp`, and the orchestrator propagates them through validation and execution so any single run can be traced back to a specific DB state.
8. **Strict grammar-constrained LLM output** - a GBNF grammar enforces the JSON schema at decode time; a confidence floor and a DB membership check gate acceptance. A separate `recovery_intent.gbnf` constrains recovery proposals to `{action, target_object_tag, target_intent_hint, rationale, confidence}`.
9. **Nav2 BT owns the recovery flow** - in `bt_led` mode the orchestrator dispatches a single `ExecutePose` goal and then does not cancel it. All path-failure detection, costmap clearing, and retry sequencing is delegated to `semantic_recovery_bt.xml`. The orchestrator only responds to explicit `/request_recovery` calls from the BT plugin (`EscalateToLLMRecovery`).
10. **Object-instance multi-disambiguation via `exclude_object_key`** - when a `retry_target` directive would resolve back to the already-blocked instance, `_resolve_target_for_directive` walks all same-tag instances and selects the nearest non-blocked one via `target_object_key` direct lookup (Precedence 1 in the resolver, bypassing caption ranking).
11. **Retry budget is BT-authoritative** - the Nav2 BT's `RetryUntilSuccessful` node controls the outer retry count; the orchestrator tracks `remaining_retry_budget` separately and returns a `give_up` directive when it reaches 0, which causes the BT's `GiveUpTerminal` subtree to execute and the `ExecutePose` action to return failure.
12. **Object-centric resolution with ranked candidates** - the resolver scores all instances of an object tag using BM25/LLM/Hybrid caption ranking and returns the highest-scoring navigable instance. The `intent_hint` string from the LLM shifts the ranking toward spatially-qualified targets (e.g. "near the window").

### Expected behavior summary

| Scenario | Behavior |
|---|---|
| Unknown query | Resolution fails, pipeline stops |
| LLM rejects command (e.g. "drive forward") | NavigatorNode returns `intent=reject`, orchestrator not invoked |
| LLM proposes object tag not in DB | NavigatorNode returns `success=false`, orchestrator not invoked |
| LLM confidence below `min_confidence_percent` | NavigatorNode returns `success=false`, orchestrator not invoked |
| Known object, unexplored area | Validation fails (`ValidateSemantic` in BT) or planner rejects |
| Path blocked mid-flight (`bt_led` mode) | `PathClearCondition` triggers, `QuerySemanticContext` gathers local objects, `EscalateToLLMRecovery` calls `/request_recovery` → orchestrator proposes `retry_target` / `wait_then_replan` / `signal_wait_recheck` / `give_up` |
| Blocked object has same-tag alternatives | `exclude_object_key` redirects directive to nearest non-blocked instance |
| Retry budget exhausted (3 attempts) | Orchestrator returns `give_up`; BT executes `GiveUpTerminal` subtree; `ExecutePose` returns failure |
| Persistent blockage (closed door) | After 3 retries all blocked, `give_up` reached cleanly |
| Human/animal blocking path | `signal_wait_recheck` directive: robot emits polite-clear signal, waits, re-checks path |
| Clearable object blocking path | `wait_then_replan` directive: robot waits `wait_seconds`, then BT replans |

---

## 16. Roadmap

1. **Live semantic DB topic ingestion** - re-add a `SemanticDB` topic subscriber to `resolver_node` that atomically replaces the snapshot at runtime, bumping `db_version`. The interfaces (`SemanticDB.msg`, the `db_version` / `db_stamp` fields on `ResolveLocation`) and the orchestrator-side propagation are already in place; only the subscriber needs to come back.
2. ~~**Recovery Orchestration with LLM**~~ - **DONE (BT-LR M1–M4, 2026-06-12).** Nav2 BT with `PathClearCondition`, `QuerySemanticContext`, `EscalateToLLMRecovery`, `EmitObstacleSignal`, and `ValidateSemantic` C++ plugins; GBNF-constrained `ProposeRecovery` endpoint; `exclude_object_key` multi-instance disambiguation; up to 3 retries with `give_up` terminal. E2E validated (Scenarios 1 & 4, 2026-06-13).
3. **E2E Scenario 2 & 3 validation** - Scenario 2: mid-flight clearable obstacle → `wait_then_replan` (requires map injection with clearable object on path). Scenario 3: human-class blockage → `signal_wait_recheck` (requires person object in map_v001.json).
4. **Optional llama_ros launch integration** - `navigator_node` is now launched by `semantic_nav_system.launch.py` by default, but the heavyweight `llama_node` model server is still launched separately. A future launch file may optionally include `llama_node` once model startup, Python environment isolation, and memory behavior are stable.
5. **Ranker evaluation with live LLM** - run the full offline eval harness (`ranker_eval`) with `llama_ros` running to capture real LLM ranker accuracy vs BM25 baseline; generate plots with `plot_ranker_eval`; populate §15.12 with real numbers.
6. **Post-failure orchestrator policies** - re-resolve / re-validate on failure, optionally gated on `db_version` change.
7. **Snapshot immutability during active navigation** - when live DB updates return, ensure an in-flight goal keeps using the snapshot it was resolved against.
