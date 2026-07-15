"""One-command full-stack E2E: ugv_rover in the AWS Small House with the semantic
navigation stack (RTAB-Map RGB-D SLAM, Nav2 + BT-led recovery, semantic core, LLM).

The ugv_rover counterpart of semantic_nav_system.launch.py: same argument surface,
same node set, same defaults — only the robot and its SLAM source differ.

  1. aws_small_house_ugv.launch.py   -> AWS world + ugv_rover spawned at origin
                                        (TB3 stack: aws_small_house_tb3.launch.py)
  2. ugv_gazebo rtabmap_rgbd         -> RTAB-Map RGB-D SLAM (/map, map->odom)
                                        (TB3 stack: rtabmap_demos turtlebot3_rgbd_scan)
  3. nav2 navigation_launch          -> Nav2 with rover_semantic_nav_params.yaml
  4. semantic_nav_core               -> resolver / validator / executor / local_object_query
  5. semantic_nav_llm (enable_llm)   -> navigator_node: /parse_semantic_command + /propose_recovery
  6. orchestrator (idle bt_led daemon) serving /navigate_to_query
  + Nav2-view RViz (/plan, /local_plan, costmaps).

SENSING: depth_only defaults to TRUE — no 2D LiDAR, /scan synthesised from the depth
camera, 59 deg of horizontal FOV. That is what the real rover (OAK-D Lite, no LiDAR)
will have on the Jetson, so this is the configuration results must be collected in.
Pass depth_only:=false to put the simulated LiDAR back for comparison.

BT-led recovery: detection lives INSIDE the Nav2 tree (semantic_recovery_bt.xml via
the semantic_nav_nav2_plugins in the params) — no external recovery-trigger monitor.
With depth_only the reobserve happens WITHOUT moving (up_front_reobserve_mode=
'dwell_then_spin' + rtabmap map_always_update), spinning only if the dwell fails.

Operator decisions are interactive by default (enable_operator_io=false): run
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('semantic_nav_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    aws_small_house_path = LaunchConfiguration('aws_small_house_path')
    world = LaunchConfiguration('world')
    depth_only = LaunchConfiguration('depth_only')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    map_always_update = LaunchConfiguration('map_always_update')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    # LLM intent parser options.
    enable_llm = LaunchConfiguration('enable_llm')
    semantic_map_path = LaunchConfiguration('semantic_map_path')
    llama_action = LaunchConfiguration('llama_action')
    parse_service = LaunchConfiguration('parse_service')
    propose_recovery_service = LaunchConfiguration('propose_recovery_service')
    grammar_path = LaunchConfiguration('grammar_path')
    recovery_grammar_path = LaunchConfiguration('recovery_grammar_path')
    recovery_max_tokens = LaunchConfiguration('recovery_max_tokens')
    min_confidence_percent = LaunchConfiguration('min_confidence_percent')
    max_tokens = LaunchConfiguration('max_tokens')
    llm_result_timeout_sec = LaunchConfiguration('llm_result_timeout_sec')
    debug_prompt = LaunchConfiguration('debug_prompt')
    debug_grammar = LaunchConfiguration('debug_grammar')

    # Operator I/O options.
    enable_operator_io = LaunchConfiguration('enable_operator_io')
    operator_auto_ack_for_dev = LaunchConfiguration('operator_auto_ack_for_dev')
    operator_prompt_timeout_sec = LaunchConfiguration('operator_prompt_timeout_sec')

    # Recovery ablation switches (A1 = deterministic baseline, A2 = LLM).
    up_front_llm_enabled = LaunchConfiguration('up_front_llm_enabled')
    open_set_inference_enabled = LaunchConfiguration('open_set_inference_enabled')

    # Rover-only: run the orchestrator as a daemon, or drive it one-shot from the CLI.
    start_orchestrator = LaunchConfiguration('start_orchestrator')

    # En-route ablation: BT XML the orchestrator dispatches with (B-LLM default,
    # or the geometric-only B-GEO variant).
    recovery_bt_xml = LaunchConfiguration('recovery_bt_xml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )

    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Run RTAB-Map in localization mode instead of SLAM mode'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial ugv_rover x position in Gazebo'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial ugv_rover y position in Gazebo'
    )

    aws_small_house_path_arg = DeclareLaunchArgument(
        'aws_small_house_path',
        default_value=os.path.expanduser(
            '/home/shaker/Thesis/Implementation/demo_bringup/aws-robomaker-small-house-world'
        ),
        description='Absolute path to aws-robomaker-small-house-world'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            bringup_dir, 'worlds', 'small_house_ugv.world'
        ),
        description='Gazebo world file. ROVER-ONLY world: the semantic scenario world '
                    'with 8 furniture models removed so the rover is not forced through '
                    'cramped spaces (its footprint + 59 deg FOV cope badly with them). '
                    'The TB3 stack keeps small_house_semantic.world untouched. Must be '
                    'used together with map_v002.json — see semantic_map_path.'
    )

    depth_only_arg = DeclareLaunchArgument(
        'depth_only', default_value='true',
        description='DEFAULT. Sense like the REAL rover: no 2D LiDAR, /scan '
                    'synthesised from the depth camera (OAK-D Lite on the hardware). '
                    'Horizontal FOV drops 360 -> 59 deg, so the rover is genuinely '
                    'blind to its sides and rear — which is the deployment reality, '
                    'and results collected any other way would not transfer to the '
                    'Jetson. Set false to put the simulated 2D LiDAR back (a sensor '
                    'the real rover does not have).'
    )

    map_always_update_arg = DeclareLaunchArgument(
        'map_always_update', default_value='true',
        description='Let RTAB-Map refresh its occupancy grid while the robot is '
                    'STATIONARY. rtabmap defaults to false, which updates the grid only '
                    'on new graph nodes (>=0.1 m / >=0.1 rad of motion) — a parked robot '
                    'then never sees a doorway clear, which is the real reason recovery '
                    'had to spin. Verified: with this true (and Grid/RayTracing) an '
                    'obstacle removed in front of the robot clears from /map in ~10 s and '
                    'from the global costmap in ~20 s, without moving.'
    )

    rtabmap_viz_arg = DeclareLaunchArgument(
        'rtabmap_viz', default_value='true',
        description="Launch the RTAB-Map GUI (rtabmap_viz): loop closures, feature "
                    "matches, the graph. Separate from the Nav2 RViz view ('rviz' arg)."
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            bringup_dir,
            'config',
            'rover_semantic_nav_params.yaml'
        ),
        description='Absolute path to the Nav2 params file (rover twin of '
                    'nav2_semantic_params.yaml; carries semantic_nav_nav2_plugins)'
    )

    enable_llm_arg = DeclareLaunchArgument(
        'enable_llm',
        default_value='true',
        description='Launch semantic_nav_llm navigator_node intent parser. '
                    'Requires llama_ros (/llama/generate_response) running separately.'
    )

    semantic_map_path_arg = DeclareLaunchArgument(
        'semantic_map_path',
        default_value=os.path.join(
            get_package_share_directory('semantic_nav_semantics'),
            'config',
            'map_v002.json',
        ),
        # ROVER-ONLY map. v002 = v001 minus the 3 entries whose backing objects were
        # deleted from small_house_ugv.world (fitness equipment, the TV cabinet, the
        # tablet) — leaving them in would let the LLM pick a target that no longer
        # exists. The TB3 stack stays on map_v001.json + small_house_semantic.world.
        # MUST match the 'world' arg: map and world are a matched pair.
        description='Absolute path to the object-centric semantic map. Rover default '
                    'is map_v002.json, which matches small_house_ugv.world.',
    )

    llama_action_arg = DeclareLaunchArgument(
        'llama_action',
        default_value='/llama/generate_response',
        description='llama_ros GenerateResponse action endpoint. On the rover the '
                    'model may run on a remote server; override if remapped.'
    )

    parse_service_arg = DeclareLaunchArgument(
        'parse_service',
        default_value='/parse_semantic_command',
        description='Service exposed by semantic_nav_llm navigator_node'
    )

    propose_recovery_service_arg = DeclareLaunchArgument(
        'propose_recovery_service',
        default_value='/propose_recovery',
        description='Service exposed by semantic_nav_llm for recovery proposals'
    )

    grammar_path_arg = DeclareLaunchArgument(
        'grammar_path',
        default_value=os.path.join(
            get_package_share_directory('semantic_nav_llm'),
            'config',
            'semantic_intent.gbnf'
        ),
        description='Absolute path to semantic intent GBNF grammar'
    )

    recovery_grammar_path_arg = DeclareLaunchArgument(
        'recovery_grammar_path',
        default_value=os.path.join(
            get_package_share_directory('semantic_nav_llm'),
            'config',
            'recovery_intent.gbnf'
        ),
        description='Absolute path to recovery GBNF grammar'
    )

    recovery_max_tokens_arg = DeclareLaunchArgument(
        'recovery_max_tokens',
        default_value='256',
        description='Request-level generation cap for recovery JSON'
    )

    min_confidence_percent_arg = DeclareLaunchArgument(
        'min_confidence_percent',
        default_value='60',
        description='Minimum confidence required for LLM navigate intent'
    )

    max_tokens_arg = DeclareLaunchArgument(
        'max_tokens',
        default_value='64',
        description='Request-level generation cap for LLM intent JSON'
    )

    llm_result_timeout_sec_arg = DeclareLaunchArgument(
        'llm_result_timeout_sec',
        default_value='180.0',
        description='Timeout waiting for llama_ros GenerateResponse result'
    )

    debug_prompt_arg = DeclareLaunchArgument(
        'debug_prompt',
        default_value='false',
        description='Print prompt sent from navigator_node to llama_ros'
    )

    debug_grammar_arg = DeclareLaunchArgument(
        'debug_grammar',
        default_value='false',
        description='Print GBNF grammar sent from navigator_node to llama_ros'
    )

    enable_operator_io_arg = DeclareLaunchArgument(
        'enable_operator_io',
        default_value='false',
        description='Launch operator_io_node. Default false: navigation_terminal serves /operator_decision.',
    )
    operator_auto_ack_for_dev_arg = DeclareLaunchArgument(
        'operator_auto_ack_for_dev',
        default_value='false',
        description='Auto-acknowledge all operator prompts without stdin (dev/CI only).',
    )
    operator_prompt_timeout_sec_arg = DeclareLaunchArgument(
        'operator_prompt_timeout_sec',
        default_value='0.0',
        description='Stdin timeout (sec) for operator prompts; 0.0 disables timeout.',
    )
    up_front_llm_enabled_arg = DeclareLaunchArgument(
        'up_front_llm_enabled',
        default_value='true',
        description='M4 ablation: true=LLM selects the up-front recovery directive '
                    '(A2); false=deterministic default only (A1).',
    )
    open_set_inference_enabled_arg = DeclareLaunchArgument(
        'open_set_inference_enabled',
        default_value='true',
        description='Open-set ablation (spec 21.4): true=LLM infers affordances for '
                    'unclassifiable blocker tags (A2); false=table-only default (A1).',
    )
    start_orchestrator_arg = DeclareLaunchArgument(
        'start_orchestrator',
        default_value='true',
        description='Run the orchestrator as an idle bt_led daemon (serves /navigate_to_query). '
                    'Set false to drive it one-shot via the CLI instead.',
    )

    recovery_bt_xml_arg = DeclareLaunchArgument(
        'recovery_bt_xml',
        default_value=os.path.join(
            get_package_share_directory('semantic_nav_nav2_plugins'),
            'config', 'semantic_recovery_bt.xml'),
        description='BT XML the orchestrator dispatches with (B-LLM default). '
                    'En-route ablation B-GEO arm: pass the installed '
                    'semantic_recovery_bt_geometric.xml path instead.',
    )

    # AWS world + ugv_rover spawn (this launch already delays its own spawn).
    aws_small_house_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'aws_small_house_ugv.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'aws_small_house_path': aws_small_house_path,
            'world': world,
            'depth_only': depth_only,
        }.items()
    )

    # RTAB-Map RGB-D SLAM — OUR launch, not ugv_gazebo's. Waveshare's hardcodes an
    # unbounded, never-cleared 3D occupancy grid (Grid/RangeMax=0, RayTracing=false)
    # that the recovery reobserve-spin smears into an unusable map. See
    # rover_rtabmap_rgbd.launch.py for the full explanation.
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'rover_rtabmap_rgbd.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
            'rtabmap_viz': rtabmap_viz,
            'map_always_update': map_always_update,
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items()
    )

    # Launch RViz2 directly (not via nav2_bringup/rviz_launch.py) so that a
    # RViz2 crash does NOT cascade-shutdown the whole stack. respawn=True
    # restarts it automatically; the nav2_bringup shutdown handler is bypassed.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    semantic_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'semantic_nav_core.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'semantic_map_path': semantic_map_path,
        }.items()
    )

    semantic_llm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'semantic_nav_llm.launch.py'
            )
        ),
        condition=IfCondition(enable_llm),
        launch_arguments={
            'semantic_map_path': semantic_map_path,
            'grammar_path': grammar_path,
            'llama_action': llama_action,
            'parse_service': parse_service,
            'min_confidence_percent': min_confidence_percent,
            'max_tokens': max_tokens,
            'llm_result_timeout_sec': llm_result_timeout_sec,
            'debug_prompt': debug_prompt,
            'debug_grammar': debug_grammar,
            'propose_recovery_service': propose_recovery_service,
            'recovery_grammar_path': recovery_grammar_path,
            'recovery_max_tokens': recovery_max_tokens,
        }.items()
    )

    # Orchestrator runs as a long-running idle service, accepting
    # /navigate_to_query goals from navigation_terminal.
    # No query arg → bt_led mode auto-sets start_idle=True.
    orchestrator_node = Node(
        package='semantic_nav_orchestrator',
        executable='navigation_orchestrator',
        name='navigation_orchestrator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'up_front_llm_enabled': up_front_llm_enabled,
            'open_set_inference_enabled': open_set_inference_enabled,
            # En-route ablation: B-LLM default XML, or the geometric-only B-GEO
            # variant passed via recovery_bt_xml:=... at launch.
            'behavior_tree': recovery_bt_xml,
            # The orchestrator loads the map under TWO params; both must point at v002
            # or it would diagnose blockages against objects the rover's world lacks.
            'semantic_map_path': semantic_map_path,
            'semantic_object_db_path': semantic_map_path,
            # ROVER-ONLY re-observe policy. The orchestrator is shared with the TB3
            # stack, whose defaults ('spin', 10 s) are left untouched — TB3's rtabmap
            # has no map_always_update, so a dwell there would stare at a frozen map.
            # Here rover_rtabmap_rgbd.launch.py DOES set map_always_update=true, so
            # the rover can re-observe a cleared doorway without moving at all, and
            # only spins if that fails.
            'up_front_reobserve_mode': 'dwell',
            # A full 2*pi turn takes this skid-steer rover ~13 s (a 90 deg Spin
            # measured 3.3 s); at the 10 s default it would ABORT part-way and leave
            # the camera pointing away from the barrier.
            'up_front_reobserve_time_allowance_s': 30.0,
        }],
        condition=IfCondition(start_orchestrator),
    )

    operator_io_node = Node(
        package='semantic_nav_operator_io',
        executable='operator_io_node',
        name='operator_io_node',
        output='screen',
        condition=IfCondition(enable_operator_io),
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_ack_for_dev': operator_auto_ack_for_dev,
            'prompt_timeout_sec': operator_prompt_timeout_sec,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        localization_arg,
        rviz_arg,
        x_pose_arg,
        y_pose_arg,
        aws_small_house_path_arg,
        world_arg,
        depth_only_arg,
        map_always_update_arg,
        rtabmap_viz_arg,
        nav2_params_file_arg,

        enable_llm_arg,
        semantic_map_path_arg,
        llama_action_arg,
        parse_service_arg,
        grammar_path_arg,
        min_confidence_percent_arg,
        max_tokens_arg,
        llm_result_timeout_sec_arg,
        debug_prompt_arg,
        debug_grammar_arg,
        propose_recovery_service_arg,
        recovery_grammar_path_arg,
        recovery_max_tokens_arg,

        enable_operator_io_arg,
        operator_auto_ack_for_dev_arg,
        operator_prompt_timeout_sec_arg,
        up_front_llm_enabled_arg,
        open_set_inference_enabled_arg,
        start_orchestrator_arg,
        recovery_bt_xml_arg,

        aws_small_house_sim_launch,

        # Longer stagger than the TB3 stack: the rover spawns from an SDF (its own
        # 3 s timer) and RTAB-Map needs the depth camera streaming before it starts.
        TimerAction(
            period=6.0,
            actions=[rtabmap_launch]
        ),

        TimerAction(
            period=9.0,
            actions=[nav2_launch, rviz_node]
        ),

        TimerAction(
            period=12.0,
            actions=[semantic_core_launch, semantic_llm_launch, operator_io_node]
        ),

        TimerAction(
            period=15.0,
            actions=[orchestrator_node]
        ),
    ])
