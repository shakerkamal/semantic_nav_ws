"""Spawn the Waveshare ugv_rover into the AWS Small House world.

Mirrors aws_small_house_tb3.launch.py but swaps TurtleBot3 for the ugv_rover so
the semantic nav stack can run on the rover in the same environment whose
coordinates the existing semantic_db.json was authored against.

Robot model split (Waveshare convention):
  - robot_state_publisher / TF  -> ugv_gazebo/urdf/ugv_rover.urdf   (via UGV_MODEL)
  - Gazebo physics + sensors    -> ugv_gazebo/models/ugv_rover/model.sdf

GAZEBO_MODEL_PATH is built here to carry BOTH the AWS house models and the ugv
models (ugv_gazebo/models for model://world+ugv_rover, ugv_description/share for
the rover link meshes referenced as model://ugv_description/...). Without the
latter the rover spawns invisible; without the former model://<aws assets> fail.

Spawn at the same origin (0,0) TurtleBot3 used so RTAB-Map's map frame aligns
with the existing AWS-house semantic_db.json coordinates.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    aws_small_house_path = LaunchConfiguration('aws_small_house_path')

    # Included Waveshare launch files read os.environ['UGV_MODEL'] at construction
    # time, so it must be set before they are evaluated (same pattern the TB3
    # launch uses for TURTLEBOT3_MODEL).
    os.environ['UGV_MODEL'] = 'ugv_rover'

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    ugv_gazebo_share = get_package_share_directory('ugv_gazebo')
    ugv_description_share = get_package_share_directory('ugv_description')

    # model.sdf carries the rover's Gazebo plugins (diff drive, depth camera, lidar, imu).
    ugv_rover_sdf = os.path.join(
        ugv_gazebo_share, 'models', 'ugv_rover', 'model.sdf'
    )

    aws_world = [
        aws_small_house_path,
        '/worlds/small_house.world',
    ]

    # Build GAZEBO_MODEL_PATH: ugv_gazebo models + ugv_description share (for the
    # rover meshes) + AWS house models + whatever is already on the path.
    gazebo_model_path = ':'.join([
        os.path.join(ugv_gazebo_share, 'models'),
        os.path.dirname(ugv_description_share),   # parent 'share' dir -> resolves model://ugv_description
        os.path.expanduser(
            '/home/shaker/Thesis/Implementation/demo_bringup/'
            'aws-robomaker-small-house-world/models'
        ),
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    ])

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': aws_world,
            'verbose': 'true',
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        )
    )

    # Rover TF: robot_state_publisher + joint_state_publisher from ugv_rover.urdf.
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ugv_gazebo_share, 'launch', 'bringup', 'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Spawn the rover from its model.sdf at the chosen pose. Delayed so gzserver
    # has advertised /spawn_entity before we call it.
    spawn_ugv_rover = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_ugv_rover',
                output='screen',
                arguments=[
                    '-entity', 'ugv_rover',
                    '-file', ugv_rover_sdf,
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose,
                    '-Y', yaw,
                ],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Initial ugv_rover x position (match TB3 origin for DB alignment)'
        ),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Initial ugv_rover y position'
        ),
        DeclareLaunchArgument(
            'z_pose', default_value='0.10',
            description='Initial ugv_rover z position (small lift to avoid floor clipping)'
        ),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Initial ugv_rover yaw'
        ),
        DeclareLaunchArgument(
            'aws_small_house_path',
            default_value=os.path.expanduser(
                '/home/shaker/Thesis/Implementation/demo_bringup/aws-robomaker-small-house-world'
            ),
            description='Absolute path to aws-robomaker-small-house-world'
        ),

        SetEnvironmentVariable('UGV_MODEL', 'ugv_rover'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_ugv_rover,
    ])
