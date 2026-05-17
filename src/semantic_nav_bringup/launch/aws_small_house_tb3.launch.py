import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    aws_small_house_path = LaunchConfiguration('aws_small_house_path')

    os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_launch_dir = os.path.join(turtlebot3_gazebo_share, 'launch')

    aws_world = [
        aws_small_house_path,
        '/worlds/small_house.world'
    ]

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': aws_world,
            'verbose': 'true'
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial Turtlebot3 x position in AWS Small House'
        ),

        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial Turtlebot3 y position in AWS Small House'
        ),

        DeclareLaunchArgument(
            'aws_small_house_path',
            default_value=os.path.expanduser(
                '/home/shaker/Thesis/Implementation/demo_bringup/aws-robomaker-small-house-world'
            ),
            description='Absolute path to aws-robomaker-small-house-world'
        ),

        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),

        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot3,
    ])