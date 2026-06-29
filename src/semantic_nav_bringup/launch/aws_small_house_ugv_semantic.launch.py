"""One-command full-stack E2E: ugv_rover in the AWS Small House with the semantic
navigation stack (RTAB-Map RGB-D SLAM, Nav2 + BT-led recovery, semantic core, LLM).

Mirrors semantic_nav_system.launch.py but for the ugv_rover. Brings up, staggered:
  1. aws_small_house_ugv.launch.py  -> AWS world + ugv_rover spawned at origin
  2. ugv_gazebo rtabmap_rgbd        -> RTAB-Map RGB-D SLAM (/map, map->odom)
  3. nav2 navigation_launch         -> Nav2, rover params WITH semantic recovery BT plugins
  4. semantic_nav_core              -> resolver / validator / executor / local_object_query
  5. semantic_nav_llm (enable_llm)  -> navigator_node: /parse_semantic_command + /propose_recovery
  6. orchestrator (idle bt_led daemon) serving /navigate_to_query
  + Nav2-view RViz (/plan, /local_plan, costmaps).

BT-led recovery: detection lives INSIDE the Nav2 tree (semantic_recovery_bt.xml via
the semantic_nav_nav2_plugins in the params) — no external recovery-trigger monitor.

Operator decisions are interactive by default (operator_mode='terminal'): run
navigation_terminal, which serves /operator_decision and prompts the human.

Drive this with the terminal (separate process, needs a TTY):
  ros2 run semantic_nav_orchestrator navigation_terminal --ros-args -p use_sim_time:=true
Type natural language (LLM) or an object key 'tag:id' (refrigerator:6, bed:78, ...).
Map an area first (teleop) — validation fails on unmapped space.

llama_ros is started separately (heavy model process); set enable_llm:=false to skip
the LLM and drive with 'tag:id' keys only.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('semantic_nav_bringup')
    ugv_gazebo_dir = get_package_share_directory('ugv_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    enable_llm = LaunchConfiguration('enable_llm')
    auto_ack_for_dev = LaunchConfiguration('auto_ack_for_dev')
    start_orchestrator = LaunchConfiguration('start_orchestrator')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Launch Nav2-view RViz')
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        # Rover params (base_footprint, real obstacle topics) WITH semantic_nav_nav2_plugins
        # added to bt_navigator.plugin_lib_names so the BT-led recovery tree loads.
        default_value=os.path.join(bringup_dir, 'config', 'rover_semantic_nav_params.yaml'),
        description='Nav2 params: rover rtabmap_dwa.yaml + semantic recovery BT plugins',
    )
    enable_llm_arg = DeclareLaunchArgument(
        'enable_llm', default_value='true',
        description='Launch navigator_node (NL parsing + LLM recovery proposals). '
                    'Requires llama_ros (/llama/generate_response) running separately.',
    )
    operator_mode_arg = DeclareLaunchArgument(
        'operator_mode', default_value='terminal',
        description="Operator decisions: 'terminal' = navigation_terminal serves "
                    "/operator_decision (human in the loop); 'auto' = operator_io_node auto-acks.",
    )
    auto_ack_for_dev_arg = DeclareLaunchArgument(
        'auto_ack_for_dev', default_value='false',
        description='Only used when operator_mode=auto: auto-acknowledge prompts (unattended/CI).',
    )
    start_orchestrator_arg = DeclareLaunchArgument(
        'start_orchestrator', default_value='true',
        description='Run the orchestrator as an idle bt_led daemon (serves /navigate_to_query). '
                    'Set false to drive it one-shot via the CLI instead.',
    )

    # 1. AWS world + ugv_rover spawn (this launch already delays its own spawn).
    aws_ugv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'aws_small_house_ugv.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    # 2. RTAB-Map RGB-D SLAM (its own RViz off; we run the Nav2 view instead).
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_gazebo_dir, 'launch', 'slam', 'rtabmap_rgbd.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
        }.items()
    )

    # 3. Nav2 navigation-only on top of RTAB-Map's /map + map->odom.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items()
    )

    # 4. Semantic core: resolver / validator / executor / local_object_query.
    semantic_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'semantic_nav_core.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 5. LLM front-end: navigator_node -> /parse_semantic_command + /propose_recovery.
    #    Uses semantic_nav_llm.launch.py defaults (map_v001.json, /llama/generate_response,
    #    GBNF grammars). llama_ros must be running for these to do real work.
    semantic_llm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'semantic_nav_llm.launch.py')
        ),
        condition=IfCondition(enable_llm),
    )

    # Optional operator I/O for the OperatorPrompt branch — only when operator_mode=auto.
    # In 'terminal' mode (default) navigation_terminal owns /operator_decision.
    operator_io_node = Node(
        package='semantic_nav_operator_io',
        executable='operator_io_node',
        name='operator_io_node',
        output='screen',
        parameters=[{
            'auto_ack_for_dev': auto_ack_for_dev,
            'prompt_timeout_sec': 0.0,
            'use_sim_time': use_sim_time,
        }],
        condition=LaunchConfigurationEquals('operator_mode', 'auto'),
    )

    # Orchestrator as an idle bt_led daemon (no query) so navigation_terminal can
    # drive it via /navigate_to_query.
    orchestrator_daemon = Node(
        package='semantic_nav_orchestrator',
        executable='navigation_orchestrator',
        name='navigation_orchestrator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'start_idle': True,
        }],
        condition=IfCondition(start_orchestrator),
    )

    # Nav2-view RViz: shows /plan (global), /local_plan (controller) and costmaps.
    # respawn so an RViz crash doesn't cascade-shutdown the stack.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        x_pose_arg,
        y_pose_arg,
        nav2_params_file_arg,
        enable_llm_arg,
        operator_mode_arg,
        auto_ack_for_dev_arg,
        start_orchestrator_arg,

        aws_ugv,
        TimerAction(period=6.0, actions=[rtabmap]),
        TimerAction(period=9.0, actions=[nav2, rviz_node]),
        TimerAction(period=12.0, actions=[semantic_core, semantic_llm, operator_io_node]),
        # Daemon after the core/services are up.
        TimerAction(period=15.0, actions=[orchestrator_daemon]),
    ])
