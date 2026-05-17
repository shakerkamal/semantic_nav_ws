import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    return LaunchDescription([
        use_sim_time_arg,
        semantic_db_path_arg,
        semantic_db_topic_arg,
        localization_arg,
        rviz_arg,
        x_pose_arg,
        y_pose_arg,
        aws_small_house_path_arg,

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
    ])