# Semantic Navigation

A modular ROS 2 Humble semantic navigation stack that allows a robot to navigate to semantically meaningful locations (e.g. "kitchen", "living room") using natural language queries. Built on top of RTAB-Map for online SLAM and Nav2 for path planning and execution, using TurtleBot3 Waffle in Gazebo simulation.

## Architecture

The system implements a three-stage deterministic pipeline:

```
User Query ("kitchen")
  -> ResolveLocation     (semantic name -> PoseStamped)
  -> ValidatePose        (ComputePathToPose reachability check)
  -> ExecutePose          (NavigateToPose execution)
```

Each stage gates the next: if resolution fails, validation and execution are skipped; if validation fails, execution is skipped.

### Package Structure

| Package | Type | Responsibility |
|---|---|---|
| `semantic_nav_interfaces` | ament_cmake | Custom messages, services, and actions |
| `semantic_nav_semantics` | ament_python | Semantic location resolution from JSON database |
| `semantic_nav_validator` | ament_python | Pre-execution path validation via Nav2 ComputePathToPose |
| `semantic_nav_executor` | ament_python | Execution bridge to Nav2 NavigateToPose |
| `semantic_nav_orchestrator` | ament_python | Pipeline coordinator (resolve -> validate -> execute) |
| `semantic_nav_bringup` | ament_python | Launch files for the integrated stack |
| `semantic_nav_llm` | ament_python | Planned LLM integration (not yet implemented) |

### Interface Definitions

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

### ROS 2 Endpoints

| Endpoint | Type | Package |
|---|---|---|
| `/resolve_location` | Service (ResolveLocation) | semantic_nav_semantics |
| `/validate_pose_goal` | Service (ValidatePose) | semantic_nav_validator |
| `/execute_pose` | Action (ExecutePose) | semantic_nav_executor |

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- TurtleBot3 packages (`turtlebot3`, `turtlebot3_simulations`)
- RTAB-Map ROS 2 (`rtabmap_ros`, `rtabmap_demos`)
- Nav2 (`navigation2`, `nav2_bringup`)
- Gazebo (Classic)

### TurtleBot3 Model

This project uses **TurtleBot3 Waffle** because the RTAB-Map RGB-D demo requires a depth-camera-capable model. Export the model environment variable:

```bash
export TURTLEBOT3_MODEL=waffle
```

The Waffle Gazebo SDF model has been patched for RTAB-Map compatibility:
- Camera sensor type changed from `camera` to `depth`
- Image resolution set to 640x480
- Optical frame (`camera_rgb_optical_frame`) added
- Frame/joint setup adjusted for RTAB-Map usage

## Installation

1. **Create a ROS 2 workspace:**
   ```bash
   mkdir -p ~/semantic_nav_ws/src
   cd ~/semantic_nav_ws
   ```

2. **Clone this repository:**
   ```bash
   git clone <repository_url> src
   ```

3. **Install dependencies:**
   ```bash
   cd ~/semantic_nav_ws
   rosdep install -i --from-path src -y --rosdistro humble
   ```

4. **Build the workspace:**
   ```bash
   cd ~/semantic_nav_ws
   colcon build
   ```

## Usage

### Workspace Sourcing

Source workspaces in this order (required for RTAB-Map resolution if it is in a separate workspace):

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_nav_ws/install/setup.bash
```

### Launch the Full System

This launches RTAB-Map (Gazebo + SLAM), the semantic resolver, validator, and executor:

```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py
```

With a custom semantic database path:

```bash
ros2 launch semantic_nav_bringup semantic_nav_system.launch.py \
  semantic_db_path:=/path/to/your/semantic_db.json
```

### Launch Core Nodes Only

For development, when RTAB-Map and Gazebo are already running separately:

```bash
ros2 launch semantic_nav_bringup semantic_nav_core.launch.py
```

### Run a Navigation Query

The orchestrator is run separately as a one-shot command:

```bash
ros2 run semantic_nav_orchestrator navigation_orchestrator --ros-args -p query:=kitchen
```

Or using positional arguments:

```bash
ros2 run semantic_nav_orchestrator navigation_orchestrator kitchen
```

### Orchestrator Parameters

| Parameter | Default | Description |
|---|---|---|
| `query` | `''` | Semantic location name to navigate to |
| `resolve_service` | `/resolve_location` | Resolution service name |
| `validate_service` | `/validate_pose_goal` | Validation service name |
| `execute_action` | `/execute_pose` | Execution action name |
| `planner_id` | `''` | Nav2 planner to use (empty = default) |
| `enable_validation` | `true` | Skip validation stage if set to false |

### Direct Service Calls

**Resolve a location:**
```bash
ros2 service call /resolve_location semantic_nav_interfaces/srv/ResolveLocation \
  "{query: 'kitchen'}"
```

**Validate a pose:**
```bash
ros2 service call /validate_pose_goal semantic_nav_interfaces/srv/ValidatePose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: '', use_start: false}"
```

## Semantic Database

The semantic database is a JSON file located at `semantic_nav_semantics/config/semantic_db.json`. It maps location names to map-frame coordinates:

```json
{
  "locations": {
    "kitchen": {
      "frame_id": "map",
      "x": 13.2217,
      "y": -0.2792,
      "yaw": 0.0,
      "aliases": ["kitchen"]
    },
    "living_room": {
      "frame_id": "map",
      "x": -2.654,
      "y": 3.220,
      "yaw": 0.0,
      "aliases": ["living room", "living space"]
    }
  }
}
```

- **`frame_id`**: Must be `"map"`. Other frames are rejected.
- **`x`, `y`**: Position in map coordinates.
- **`yaw`**: Orientation in radians (converted to quaternion internally).
- **`aliases`**: Alternative names for the location. The resolver normalizes queries (lowercase, underscores to spaces, trimmed whitespace) and matches against both location IDs and aliases.

### Important Constraint

Semantic navigation only works for locations that are **already in mapped, navigable free space**. Because RTAB-Map runs in online/live mapping mode, goals in unexplored areas will cause validation or execution to fail. This is expected behavior, not a bug.

## Design Principles

1. **Modular ROS 2 package separation** - each concern in its own package
2. **No direct LLM-to-motion coupling** - LLM output will be structured semantic commands, not raw motion
3. **Validation before execution** - planner-based reachability check before committing to navigation
4. **Stage-specific logging and failure isolation** - each pipeline stage logs with `[RESOLUTION]`, `[VALIDATION]`, or `[EXECUTION]` prefix
5. **Live RTAB-Map map, not static map file** - Nav2 consumes the live `/map` topic
6. **Semantic navigation only for explored regions** - goals must be in mapped free space

## Pipeline Behavior

| Scenario | Result |
|---|---|
| Unknown location query | Resolution fails, pipeline stops |
| Known location but outside mapped area | Validation fails, execution skipped |
| Validation passes but dynamic obstacle blocks path | Execution may abort after Nav2 recovery attempts |
| Temporary obstacle (e.g. person) | Nav2 may replan and succeed |
| Persistent blockage (e.g. closed door) | Validation or execution fails |

## Next Steps

1. **Live semantic DB subscriber with versioned snapshots** - subscribe to a topic for live database updates with snapshot versioning and `db_version` propagation through the orchestrator
2. **Post-failure orchestrator policies** - re-resolve or re-validate after failures, optionally comparing database versions
3. **LLM integration** (`semantic_nav_llm`) - use LLAMA-ROS with LLaMA 3 8B GGUF for natural language intent resolution into structured semantic commands
