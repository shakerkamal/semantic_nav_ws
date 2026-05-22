import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = get_package_share_directory('semantic_nav_bringup')
    rtabmap_demos_dir = get_package_share_directory('rtabmap_demos')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    semantic_db_path = LaunchConfiguration('semantic_db_path')
    semantic_db_topic = LaunchConfiguration('semantic_db_topic')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    aws_small_house_path = LaunchConfiguration('aws_small_house_path')

    # LLM intent parser options.
    enable_llm = LaunchConfiguration('enable_llm')
    llm_semantic_db_path = LaunchConfiguration('llm_semantic_db_path')
    llama_action = LaunchConfiguration('llama_action')
    parse_service = LaunchConfiguration('parse_service')
    grammar_path = LaunchConfiguration('grammar_path')
    min_confidence_percent = LaunchConfiguration('min_confidence_percent')
    max_tokens = LaunchConfiguration('max_tokens')
    llm_result_timeout_sec = LaunchConfiguration('llm_result_timeout_sec')
    debug_prompt = LaunchConfiguration('debug_prompt')
    debug_grammar = LaunchConfiguration('debug_grammar')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )

    semantic_db_path_arg = DeclareLaunchArgument(
        'semantic_db_path',
        default_value='',
        description='Optional absolute path to semantic_db.json'
    )

    semantic_db_topic_arg = DeclareLaunchArgument(
        'semantic_db_topic',
        default_value='/semantic_nav/semantic_database',
        description='Topic for live semantic database snapshots'
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

    enable_llm_arg = DeclareLaunchArgument(
        'enable_llm',
        default_value='true',
        description='Launch semantic_nav_llm navigator_node intent parser'
    )

    llm_semantic_db_path_arg = DeclareLaunchArgument(
        'llm_semantic_db_path',
        default_value=os.path.join(
            get_package_share_directory('semantic_nav_semantics'),
            'config',
            'semantic_db.json'
        ),
        description='Absolute path to semantic_db.json used by semantic_nav_llm navigator_node'
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

    grammar_path_arg = DeclareLaunchArgument(
        'grammar_path',
        default_value=os.path.join(
            get_package_share_directory('semantic_nav_llm'),
            'config',
            'semantic_intent.gbnf'
        ),
        description='Absolute path to semantic intent GBNF grammar'
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
        default_value='60.0',
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

    nav2_params_file = os.path.join(
        rtabmap_demos_dir,
        'params',
        'turtlebot3_rgbd_scan_nav2_params.yaml'
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

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'rviz_launch.py'
            )
        ),
        condition=IfCondition(rviz)
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
            'semantic_db_path': semantic_db_path,
            'semantic_db_topic': semantic_db_topic,
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
            'semantic_db_path': llm_semantic_db_path,
            'grammar_path': grammar_path,
            'llama_action': llama_action,
            'parse_service': parse_service,
            'min_confidence_percent': min_confidence_percent,
            'max_tokens': max_tokens,
            'llm_result_timeout_sec': llm_result_timeout_sec,
            'debug_prompt': debug_prompt,
            'debug_grammar': debug_grammar,
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        semantic_db_path_arg,
        semantic_db_topic_arg,
        localization_arg,
        rviz_arg,
        x_pose_arg,
        y_pose_arg,
        aws_small_house_path_arg,

        enable_llm_arg,
        llm_semantic_db_path_arg,
        llama_action_arg,
        parse_service_arg,
        grammar_path_arg,
        min_confidence_percent_arg,
        max_tokens_arg,
        llm_result_timeout_sec_arg,
        debug_prompt_arg,
        debug_grammar_arg,

        aws_small_house_sim_launch,

        TimerAction(
            period=3.0,
            actions=[rtabmap_launch]
        ),

        TimerAction(
            period=5.0,
            actions=[nav2_launch, rviz_launch]
        ),

        semantic_core_launch,
        semantic_llm_launch,
    ])