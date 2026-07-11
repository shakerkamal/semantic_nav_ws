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
    rtabmap_demos_dir = get_package_share_directory('rtabmap_demos')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    aws_small_house_path = LaunchConfiguration('aws_small_house_path')
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
        description='Initial TurtleBot3 x position in Gazebo'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial TurtleBot3 y position in Gazebo'
    )

    aws_small_house_path_arg = DeclareLaunchArgument(
        'aws_small_house_path',
        default_value=os.path.expanduser(
            '/home/shaker/Thesis/Implementation/demo_bringup/aws-robomaker-small-house-world'
        ),
        description='Absolute path to aws-robomaker-small-house-world'
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            bringup_dir,
            'config',
            'nav2_semantic_params.yaml'
        ),
        description='Absolute path to the Nav2 params file'
    )

    enable_llm_arg = DeclareLaunchArgument(
        'enable_llm',
        default_value='true',
        description='Launch semantic_nav_llm navigator_node intent parser'
    )

    semantic_map_path_arg = DeclareLaunchArgument(
        'semantic_map_path',
        default_value=os.path.join(
            get_package_share_directory('semantic_nav_semantics'),
            'config',
            'map_v001.json',
        ),
        description='Absolute path to object-centric semantic map_v001.json',
    )

    llama_action_arg = DeclareLaunchArgument(
        'llama_action',
        default_value='/llama/generate_response',
        description='llama_ros GenerateResponse action endpoint'
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

    aws_small_house_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'aws_small_house_tb3.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'aws_small_house_path': aws_small_house_path,
        }.items()
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rtabmap_demos_dir,
                'launch',
                'turtlebot3',
                'turtlebot3_rgbd_scan.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
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
            'up_front_llm_enabled': up_front_llm_enabled,
            'open_set_inference_enabled': open_set_inference_enabled,
        }],
    )

    operator_io_node = Node(
        package='semantic_nav_operator_io',
        executable='operator_io_node',
        name='operator_io_node',
        output='screen',
        condition=IfCondition(enable_operator_io),
        parameters=[{
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

        aws_small_house_sim_launch,

        TimerAction(
            period=3.0,
            actions=[rtabmap_launch]
        ),

        TimerAction(
            period=5.0,
            actions=[nav2_launch, rviz_node]
        ),

        semantic_core_launch,
        semantic_llm_launch,
        operator_io_node,

        TimerAction(
            period=10.0,
            actions=[orchestrator_node]
        ),
    ])
