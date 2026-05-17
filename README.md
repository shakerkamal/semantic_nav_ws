# LLM Navigator

A modular ROS 2 Humble semantic navigation stack that lets a robot drive to semantically named locations (e.g. `"kitchen"`, `"living room"`) instead of raw coordinates. Built on top of RTAB-Map for online SLAM and Nav2 for planning/execution, running a TurtleBot3 Waffle in the **AWS RoboMaker Small House** Gazebo world (chosen because RTAB-Map's visual loop closure needs a texture-rich environment).

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
9. [Semantic database](#9-semantic-database)
10. [Launch arguments reference](#10-launch-arguments-reference)
11. [ROS 2 endpoints](#11-ros-2-endpoints)
12. [Interface definitions](#12-interface-definitions)
13. [Troubleshooting](#13-troubleshooting)
14. [Design principles](#14-design-principles)
15. [Roadmap](#15-roadmap)

---

## 1. What this project does

Given a natural language location query (`I am tired.`, `I am hungry.`, etc.), the system:

1. **Resolves** the name to a `PoseStamped` in the `map` frame using a JSON database.
2. **Validates** that Nav2's planner can currently compute a path to that pose (catches goals in unmapped or blocked areas before committing).
3. **Executes** the navigation via Nav2's `NavigateToPose`.

Each stage gates the next: if resolution fails, validation/execution are skipped; if validation fails, execution is skipped. RTAB-Map provides live SLAM (no pre-saved map), so semantic destinations must already be in the explored, navigable free space at the time you issue the query.

---

## 2. System architecture

### Pipeline

```
User intent ("I am hungry")
    │
    ▼
LLM resolves the intent ("kitchen")
    │
    ▼
ResolveLocation  (semantic name → PoseStamped)
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
| `semantic_nav_semantics`  | ament_python | Semantic name → pose resolution (JSON DB + live topic) |
| `semantic_nav_validator`  | ament_python | Path-existence check via `ComputePathToPose` |
| `semantic_nav_executor`   | ament_python | Bridges custom `ExecutePose` action to Nav2 `NavigateToPose` |
| `semantic_nav_orchestrator` | ament_python | One-shot pipeline runner (resolve → validate → execute) |
| `semantic_nav_bringup`    | ament_python | Launch files for the integrated stack |
| `semantic_nav_llm`        | ament_python | LLM intent layer (planned, not implemented) |

---

## 3. Required workspaces and packages

This project lives across **two workspaces** that both need to be sourced at runtime:

| Workspace | Holds | Built? |
|---|---|---|
| `~/demo_bringup` | RTAB-Map ROS 2 sources (in `src/`) and a cloned `aws-robomaker-small-house-world/` as a sibling of `src/` | RTAB-Map yes (colcon). AWS world **no** — we only consume its `worlds/` and `models/` assets |
| `~/semantic_nav_ws` | **This repo** — all `semantic_nav_*` packages | Yes |

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

These steps assume you have Ubuntu 22.04 and ROS 2 Humble installed. If you have never sourced ROS 2 before, run `source /opt/ros/humble/setup.bash` once in any new terminal you open from now on — every command below assumes ROS 2 is sourced.

### 4.1 Clone this repo

```bash
mkdir -p ~/semantic_nav_ws/src
cd ~/semantic_nav_ws/src
git clone <THIS_REPO_URL> .
```

### 4.2 Set up `~/demo_bringup` (RTAB-Map sources + AWS world assets)

```bash
mkdir -p ~/demo_bringup/src

# RTAB-Map sources — will be built with colcon
cd ~/demo_bringup/src
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git
git clone https://github.com/introlab/rtabmap.git

# AWS small house — cloned as a sibling of src/, NOT built
cd ~/demo_bringup
git clone --branch ros2 https://github.com/aws-robotics/aws-robomaker-small-house-world.git
```

The AWS small house package is **not** a buildable ROS dependency for our setup — we only use its `worlds/small_house.world` file and `models/` directory. Cloning it outside `src/` keeps colcon from trying to build it, which avoids pulling in `gazebo_ros` as a build-time dependency and keeps the workspace clean.

### 4.3 Patch the TurtleBot3 Waffle SDF (required by RTAB-Map)

The stock TurtleBot3 Waffle has an RGB camera. RTAB-Map's demo expects a **depth** camera with an optical frame. Create a backup of the `model.sdf` file and then edit
`/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf`:

1. Change the camera sensor type from `<sensor name="camera" type="camera">` → `<sensor name="camera" type="depth">`.
2. Change image resolution from `1920x1080` → `640x480`.
3. Rename `<link name="camera_rgb_frame">` → `<link name="camera_rgb_optical_frame">`.
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
6. (Optional, recommended) Increase the lidar min range from `0.12` → `0.2` to avoid self-hits.

The same patch is documented in the header comment of `/opt/ros/humble/share/rtabmap_demos/launch/turtlebot3/turtlebot3_sim_rgbd_scan_demo.launch.py` — refer to it if anything is unclear.

---

## 5. Building

Build the two workspaces **in this order** (RTAB-Map first, then this repo):

```bash
# 1. RTAB-Map (only the rtabmap_* packages inside demo_bringup/src are built)
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

The `aws-robomaker-small-house-world/` directory at `~/demo_bringup/` is intentionally outside `src/`, so colcon will ignore it. If any build fails with "package not found", you almost certainly forgot to source a prior workspace before building the next one.

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


**Why `GAZEBO_MODEL_PATH` matters:** the AWS world references its 60+ furniture meshes via `model://aws_robomaker_residential_*` URIs. Gazebo resolves those URIs by scanning `GAZEBO_MODEL_PATH`. The AWS world is asset-only (not built), so nothing exports `GAZEBO_MODEL_PATH` for you — you must set it manually or the Gazebo window opens empty and you get dozens of `Error Code 12 Msg: Unable to find uri[model://...]` errors.

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

Open a new terminal and source the two workspaces in this order, then launch:

```bash
source /opt/ros/humble/setup.bash
source ~/demo_bringup/install/setup.bash
source ~/semantic_nav_ws/install/setup.bash

ros2 launch semantic_nav_bringup semantic_nav_system.launch.py
```

### What happens

The launch starts components in a staggered order to avoid race conditions:

| t (s) | What starts |
|---|---|
| 0 | Gazebo (gzserver + gzclient) loads the AWS small house, robot_state_publisher, spawns TurtleBot3 Waffle |
| 0 | `semantic_resolver`, `semantic_nav_validator`, `semantic_nav_executor` nodes |
| 3 | RTAB-Map (`turtlebot3_rgbd_scan.launch.py`) |
| 5 | Nav2 (`navigation_launch.py`) and RViz |

You should see Gazebo render the textured house, then RViz appear with a growing occupancy grid as RTAB-Map builds the map.

### Driving the robot manually (to explore the map)

Open a **second** terminal (with `/opt/ros/humble/setup.bash` sourced) and run:

```bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

Drive the robot through rooms until the area containing your target location is visible in RViz's occupancy grid. Semantic navigation will fail to validate goals in unexplored space.

---

## 8. Sending a navigation query

Open a **third** terminal, source the same three scripts as in Section 7, then:

```bash
ros2 run semantic_nav_orchestrator navigation_orchestrator --ros-args -p query:=kitchen
```

Or use the positional shorthand:

```bash
ros2 run semantic_nav_orchestrator navigation_orchestrator kitchen
```

The orchestrator logs each stage:

```
[RESOLUTION] Resolving location for query: "kitchen"
[RESOLUTION] Resolved 'kitchen' -> location_id='kitchen', ...
[VALIDATION] Path found, length=...
[EXECUTION] NavigateToPose succeeded
```

If a stage fails, the prefix tells you exactly which one (`[RESOLUTION]`, `[VALIDATION]`, `[EXECUTION]`).

### Orchestrator parameters

| Parameter | Default | Description |
|---|---|---|
| `query` | `''` | Semantic location name to navigate to |
| `resolve_service` | `/resolve_location` | Resolution service name |
| `validate_service` | `/validate_pose_goal` | Validation service name |
| `execute_action` | `/execute_pose` | Execution action name |
| `planner_id` | `''` | Nav2 planner to use (empty = default) |
| `enable_validation` | `true` | Skip validation stage if set to `false` |

---

## 9. Semantic database

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
| `frame_id` | Must be `"map"`. Other frames are rejected. |
| `x`, `y` | Position in meters, in the map frame. |
| `yaw` | Heading in radians (converted to quaternion internally). |
| `aliases` | Alternative names. Queries are normalized (lowercase, underscores → spaces, trimmed) and matched against both the key and the aliases. |

### Coordinates are world-specific

The coordinates checked into the repo were measured for the **AWS Small House**. If you swap worlds you will need to redo them — drive the robot to each room in Gazebo, read its `/odom` or RViz pose, and update the JSON.

### Overriding the DB at launch time

```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py \
  semantic_db_path:=/absolute/path/to/your_db.json
```

### Live DB updates

`semantic_nav_semantics` also subscribes to a topic (default `/semantic_nav/semantic_database`) for runtime DB updates. The in-memory snapshot is replaced atomically; an active navigation keeps its originally resolved pose to avoid mid-flight target switches.

---

## 10. Launch arguments reference

### `semantic_nav_system.launch.py`

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use the Gazebo simulation clock |
| `semantic_db_path` | `''` (use package default) | Absolute path to a `semantic_db.json` override |
| `semantic_db_topic` | `/semantic_nav/semantic_database` | Topic for live DB snapshots |
| `localization` | `false` | Run RTAB-Map in localization mode instead of SLAM |
| `rviz` | `true` | Launch RViz |
| `x_pose` | `0.0` | Initial TurtleBot3 X position in Gazebo |
| `y_pose` | `0.0` | Initial TurtleBot3 Y position in Gazebo |
| `aws_small_house_path` | `~/demo_bringup/aws-robomaker-small-house-world` | Path to the AWS world source dir (must contain `worlds/small_house.world`) |

**If you cloned the AWS package elsewhere**, you must pass `aws_small_house_path:=/your/path` on the launch command line — or edit the default in `src/semantic_nav_bringup/launch/semantic_nav_system.launch.py`.

---

## 11. ROS 2 endpoints

| Endpoint | Type | Provider |
|---|---|---|
| `/resolve_location` | Service (`semantic_nav_interfaces/srv/ResolveLocation`) | `semantic_nav_semantics` |
| `/validate_pose_goal` | Service (`semantic_nav_interfaces/srv/ValidatePose`) | `semantic_nav_validator` |
| `/execute_pose` | Action (`semantic_nav_interfaces/action/ExecutePose`) | `semantic_nav_executor` |
| `/semantic_nav/semantic_database` | Topic (`semantic_nav_interfaces/msg/SemanticDB`) | external publisher (optional) |

### Direct service calls (handy for debugging without the orchestrator)

```bash
# Resolve a name
ros2 service call /resolve_location semantic_nav_interfaces/srv/ResolveLocation \
  "{query: 'kitchen'}"

# Validate a pose
ros2 service call /validate_pose_goal semantic_nav_interfaces/srv/ValidatePose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: '', use_start: false}"
```

---

## 12. Interface definitions

**`ResolveLocation.srv`**
```
string query
---
bool success
string location_id
geometry_msgs/PoseStamped pose
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

## 13. Troubleshooting

### Gazebo shows an empty house, log spams `Error Code 12 Msg: Unable to find uri[model://aws_robomaker_residential_*]`
`GAZEBO_MODEL_PATH` does not include the AWS models directory. See [Section 6](#6-environment-variables-critical). The most common cause is a `~/.bashrc` line that uses `=` instead of `:` to set the path, clobbering it on shell startup.

### `[Err] [RenderEngine.cc:197] Failed to initialize scene` followed by a null-`Scene`/`Camera` boost assert
A previous `gzserver`/`gzclient` is still alive and holding the X display. Kill them and re-run:
```bash
pkill -9 gzserver; pkill -9 gzclient
```
If the error persists on a fresh boot, suspect a GPU/Ogre problem. Confirm with `glxinfo -B` (install `mesa-utils` if missing) and try `LIBGL_ALWAYS_SOFTWARE=1` as a sanity check.

### `Package 'rtabmap_demos' not found` (or `aws_robomaker_small_house_world`)
You forgot to source one of the workspaces. Re-do the three `source` commands in [Section 7](#7-running-the-system), in order. If `rtabmap_demos` is missing entirely, you have not built `~/demo_bringup` — see [Section 5](#5-building).

### Validation fails for a known location ("kitchen")
The robot has not yet explored the area containing that location. Drive the robot with teleop until the room is visible in RViz's occupancy grid, then retry the query. This is by design — see Section 14, principle 6.

### Robot spawns inside a wall
The default `x_pose=0.0 y_pose=0.0` may not be in free space depending on AWS house origin. Pass spawn coordinates that you have visually confirmed are clear:
```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py x_pose:=-2.0 y_pose:=0.5
```

---

## 14. Design principles

1. **Modular ROS 2 package separation** — each concern in its own package.
2. **No direct LLM-to-motion coupling** — when the LLM layer is added it will emit structured semantic commands, never raw motion.
3. **Validation before execution** — planner-based reachability check before committing to navigation.
4. **Stage-specific logging and failure isolation** — `[RESOLUTION]` / `[VALIDATION]` / `[EXECUTION]` prefixes.
5. **Live RTAB-Map map** — Nav2 consumes the live `/map` topic, no pre-saved map file.
6. **Semantic navigation only for explored regions** — goals must be in mapped free space at the moment the query is issued.
7. **Snapshot immutability during active navigation** — a live DB update never mutates a goal that is already being executed.

### Expected behavior summary

| Scenario | Behavior |
|---|---|
| Unknown query | Resolution fails, pipeline stops |
| Known location, unexplored area | Validation fails, execution skipped |
| Path passes validation, dynamic obstacle blocks later | Nav2 recovery may succeed; if not, execution aborts |
| Persistent blockage (closed door) | Validation or execution fails |
| Live DB update mid-navigation | Snapshot replaced atomically; active goal unchanged |

---

## 15. Roadmap

1. **Live semantic DB with versioned snapshots and `db_version` propagation** — implemented; orchestrator logs the snapshot version through all three stages.
2. **Post-failure orchestrator policies** — re-resolve / re-validate on failure, optionally gated on `db_version` change.
3. **LLM integration (`semantic_nav_llm`)** — LLAMA-ROS + LLaMA 3 8B GGUF for natural-language intent → structured semantic command.

Target future pipeline:

```
Natural language
  → LLAMA-ROS
  → structured semantic intent
  → ResolveLocation
  → ValidatePose
  → ExecutePose
```
