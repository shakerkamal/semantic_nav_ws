# LLM Navigator

A modular ROS 2 Humble semantic navigation stack that lets a robot drive to semantically named locations (e.g. `"kitchen"`, `"living room"`) instead of raw coordinates, and accepts free-form natural-language commands (e.g. `"I am hungry"`) that an LLM translates into a constrained semantic intent. Built on top of RTAB-Map for online SLAM and Nav2 for planning/execution, running a TurtleBot3 Waffle in the **AWS RoboMaker Small House** Gazebo world (chosen because RTAB-Map's visual loop closure needs a texture-rich environment).

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

Given a semantic location query (`kitchen`, `living room`) - or a natural-language command that an LLM converts into one - the system:

1. **Parses** (optional) a natural-language command via `semantic_nav_llm` into a constrained `{action, target, confidence}` JSON intent enforced by a GBNF grammar.
2. **Resolves** the target name to a `PoseStamped` in the `map` frame using a JSON database, and stamps the response with the active `db_version`.
3. **Validates** that Nav2's planner can currently compute a path to that pose (catches goals in unmapped or blocked areas before committing).
4. **Executes** the navigation via Nav2's `NavigateToPose`.

Each stage gates the next: if parsing/resolution fails, downstream stages are skipped; if validation fails, execution is skipped. RTAB-Map provides live SLAM (no pre-saved map), so semantic destinations must already be in the explored, navigable free space at the time you issue the query.

---

## 2. System architecture

### Pipeline

```
User intent ("I am hungry")
    │
    ▼
NavigatorNode (LLM + GBNF) -> {action: navigate, target: "kitchen", confidence: 92}
    │  (or a direct semantic query, skipping the LLM step)
    ▼
ResolveLocation  (semantic name -> PoseStamped, db_version, db_stamp)
    │
    ▼
ValidatePose    (Nav2 ComputePathToPose feasibility check)
    │
    ▼
ExecutePose     (Nav2 NavigateToPose execution)
    │
    ▼
Robot moves
```

### Package layout (this workspace, `src/`)

| Package | Type | Responsibility |
|---|---|---|
| `semantic_nav_interfaces` | ament_cmake | Custom messages, services, actions |
| `semantic_nav_semantics`  | ament_python | Semantic name -> pose resolution (JSON DB, versioned snapshot) |
| `semantic_nav_validator`  | ament_python | Path-existence check via `ComputePathToPose` |
| `semantic_nav_executor`   | ament_python | Bridges custom `ExecutePose` action to Nav2 `NavigateToPose` |
| `semantic_nav_orchestrator` | ament_python | One-shot pipeline runner (resolve -> validate -> execute), propagates `db_version` |
| `semantic_nav_llm`        | ament_python | Natural-language -> constrained semantic intent (llama_ros + GBNF) |
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

The database is a JSON file at `src/semantic_nav_semantics/config/semantic_db.json`. Each entry maps a location key to a pose in the `map` frame, plus aliases:

```json
{
  "locations": {
    "kitchen": {
      "frame_id": "map",
      "x": 13.2217, "y": -0.2792, "yaw": 0.0,
      "aliases": ["kitchen"]
    },
    "living_room": {
      "frame_id": "map",
      "x": -2.6538, "y": 3.2202, "yaw": 0.0,
      "aliases": ["living room", "living space"]
    }
  }
}
```

| Field | Rules |
|---|---|
| `frame_id` | Must be `"map"`. Other frames are rejected at load time. |
| `x`, `y` | Position in meters, in the map frame. Must be finite. |
| `yaw` | Heading in radians (converted to quaternion internally). |
| `aliases` | Alternative names. Queries are normalized (lowercase, underscores -> spaces, trimmed) and matched against both the key and the aliases. Alias collisions across different locations are rejected at load time. |

### Versioning

The resolver builds an **immutable snapshot** at startup and stamps every `ResolveLocation` response with `db_version` (default `1`, configurable via the `initial_db_version` parameter on `resolver_node`) and `db_stamp` (the file mtime). The orchestrator carries these fields through resolution -> validation -> execution log lines, so you can correlate which DB snapshot a given run used. This is plumbing for the planned live-update path; the current resolver does not yet subscribe to a live topic (see Section 16).

### Coordinates are world-specific

The coordinates checked into the repo were measured for the **AWS Small House**. If you swap worlds you will need to redo them - drive the robot to each room in Gazebo, read its `/odom` or RViz pose, and update the JSON.

### Overriding the DB at launch time

```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py \
  semantic_db_path:=/absolute/path/to/your_db.json
```

If you change the DB while the system is running, restart the `semantic_resolver` (and `navigator_node` if you use the LLM) to reload it.

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

| Endpoint | Type | Provider |
|---|---|---|
| `/resolve_location` | Service (`semantic_nav_interfaces/srv/ResolveLocation`) | `semantic_nav_semantics` |
| `/validate_pose_goal` | Service (`semantic_nav_interfaces/srv/ValidatePose`) | `semantic_nav_validator` |
| `/execute_pose` | Action (`semantic_nav_interfaces/action/ExecutePose`) | `semantic_nav_executor` |
| `/parse_semantic_command` | Service (`semantic_nav_interfaces/srv/ParseSemanticCommand`) | `semantic_nav_llm` (optional) |
| `/llama/generate_response` | Action (`llama_msgs/action/GenerateResponse`) | external (`llama_ros`), consumed by `semantic_nav_llm` |

### Direct service calls (handy for debugging without the orchestrator)

```bash
# Resolve a name
ros2 service call /resolve_location semantic_nav_interfaces/srv/ResolveLocation \
  "{query: 'kitchen'}"

# Validate a pose
ros2 service call /validate_pose_goal semantic_nav_interfaces/srv/ValidatePose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: '', use_start: false}"

# Parse a natural-language command (requires navigator_node + llama_ros running)
ros2 service call /parse_semantic_command semantic_nav_interfaces/srv/ParseSemanticCommand \
  "{command: 'I am hungry'}"
```

---

## 13. Interface definitions

**`ResolveLocation.srv`**
```
string query
---
bool success
string location_id
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
string location_query        # raw target string from the LLM (post-normalization)
string canonical_location_id # location_id matched in semantic_db.json (empty if not navigate)
uint8 confidence_percent
bool location_known
string raw_output            # the JSON the LLM produced under the GBNF grammar
string message
```

**`ExecutePose.action`**
```
# Goal
geometry_msgs/PoseStamped pose
string behavior_tree
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
2. **No direct LLM-to-motion coupling** - the LLM emits a structured `{action, target, confidence}` intent only. Targets are validated against the semantic DB; the LLM cannot fabricate coordinates, motion commands, behavior trees, or unknown rooms.
3. **Validation before execution** - planner-based reachability check before committing to navigation.
4. **Stage-specific logging and failure isolation** - `[LLM_INTENT]` / `[RESOLUTION]` / `[VALIDATION]` / `[EXECUTION]` prefixes.
5. **Live RTAB-Map map** - Nav2 consumes the live `/map` topic, no pre-saved map file.
6. **Semantic navigation only for explored regions** - goals must be in mapped free space at the moment the query is issued.
7. **Versioned semantic snapshots** - every resolution stamps its response with `db_version` and `db_stamp`, and the orchestrator propagates them through validation and execution so any single run can be traced back to a specific DB state.
8. **Strict grammar-constrained LLM output** - a GBNF grammar enforces the JSON schema at decode time; a confidence floor and a DB membership check gate acceptance.

### Expected behavior summary

| Scenario | Behavior |
|---|---|
| Unknown query | Resolution fails, pipeline stops |
| LLM rejects command (e.g. "drive forward") | NavigatorNode returns `intent=reject`, orchestrator not invoked |
| LLM proposes target not in DB | NavigatorNode returns `success=false`, orchestrator not invoked |
| LLM confidence below `min_confidence_percent` | NavigatorNode returns `success=false`, orchestrator not invoked |
| Known location, unexplored area | Validation fails, execution skipped |
| Path passes validation, dynamic obstacle blocks later | Nav2 recovery may succeed; if not, execution aborts |
| Persistent blockage (closed door) | Validation or execution fails |

---

## 16. Roadmap

1. **Live semantic DB topic ingestion** - re-add a `SemanticDB` topic subscriber to `resolver_node` that atomically replaces the snapshot at runtime, bumping `db_version`. The interfaces (`SemanticDB.msg`, the `db_version` / `db_stamp` fields on `ResolveLocation`) and the orchestrator-side propagation are already in place; only the subscriber needs to come back.
2. **Recovery Orchestration with LLM** - a recovery layer in which orchestrator failures call back into `navigator_node` to get an LLM-proposed alternative, autonomously, capped, with user-intervention escalation.
3. **Optional llama_ros launch integration** - `navigator_node` is now launched by `semantic_nav_system.launch.py` by default, but the heavyweight `llama_node` model server is still launched separately. A future launch file may optionally include `llama_node` once model startup, Python environment isolation, and memory behavior are stable.
4. **Post-failure orchestrator policies** - re-resolve / re-validate on failure, optionally gated on `db_version` change.
5. **Validator-feedback regeneration loop** - optionally reuse the earlier agent pattern where `UNKNOWN_TARGET`, `AMBIGUOUS_TARGET`, or `NO_PATH` feedback can trigger one bounded LLM retry with constrained options.
6. **Snapshot immutability during active navigation** - when live DB updates return, ensure an in-flight goal keeps using the snapshot it was resolved against.
