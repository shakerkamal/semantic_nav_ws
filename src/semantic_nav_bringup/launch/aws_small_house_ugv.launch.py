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

Loads the SAME world as aws_small_house_tb3.launch.py by default — our baked
small_house_semantic.world, not the stock AWS small_house.world — so the rover
sees the scenario geometry (closed-door walls at door:119) the recovery
experiments depend on.
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    world = LaunchConfiguration('world')
    depth_only = LaunchConfiguration('depth_only')
    aws_small_house_path = LaunchConfiguration('aws_small_house_path')

    # Included Waveshare launch files read os.environ['UGV_MODEL'] at construction
    # time, so it must be set before they are evaluated (same pattern the TB3
    # launch uses for TURTLEBOT3_MODEL).
    os.environ['UGV_MODEL'] = 'ugv_rover'

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    ugv_gazebo_share = get_package_share_directory('ugv_gazebo')
    ugv_description_share = get_package_share_directory('ugv_description')

    bringup_share = get_package_share_directory('semantic_nav_bringup')

    # model.sdf carries the rover's Gazebo plugins (diff drive, depth camera, lidar, imu).
    # depth_only swaps in our copy with the 2D LiDAR sensor stripped out, so the sim
    # rover senses exactly what the real OAK-D rover does. /scan is then synthesised
    # from the depth image by depthimage_to_laserscan below.
    ugv_rover_sdf = PythonExpression([
        "'", os.path.join(bringup_share, 'models', 'ugv_rover_depth', 'model.sdf'), "'",
        " if '", depth_only, "'.lower() in ('true','1') else ",
        "'", os.path.join(ugv_gazebo_share, 'models', 'ugv_rover', 'model.sdf'), "'",
    ])

    # 3d_camera_link is an OPTICAL frame: RPY(-90, 0, -90), i.e. +Z forward,
    # +X right, +Y down. A LaserScan sweeps its frame's XY plane about +Z, so
    # emitting the scan in an optical frame makes it a VERTICAL fan rotating about
    # the forward axis — in the 2D grid that shows up as radial spokes, not walls.
    #
    # depthimage_to_laserscan expects a NON-optical output_frame (its own default is
    # 'camera_depth_frame'). So publish camera_scan_link at the camera's position with
    # the optical rotation undone (conjugate quaternion), giving x-forward / z-up.
    # Verified: base_footprint -> camera_scan_link is exactly identity rotation.
    camera_scan_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_scan_link_tf',
        output='screen',
        condition=IfCondition(depth_only),
        arguments=[
            '--frame-id', '3d_camera_link',
            '--child-frame-id', 'camera_scan_link',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Depth -> LaserScan. The rover's RGB and depth come from ONE Gazebo sensor
    # (type="depth" in link 3d_camera_link), so they share intrinsics and
    # /camera/camera_info is the correct camera_info for the depth image —
    # there is no /camera/depth/camera_info topic.
    #
    # NOTE the FOV collapse: the depth camera is 1.02974 rad = 59 deg, versus the
    # LiDAR's 360. The rover becomes blind to its sides and rear, which is exactly
    # the real robot's situation.
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        condition=IfCondition(depth_only),
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_frame': 'camera_scan_link',
            'range_min': 0.2,
            # 8.0, NOT the 3.5 this used to be. 3.5 was the Waveshare LiDAR's
            # <max>3.5</max> — and in depth_only there IS no LiDAR, so that ceiling
            # was a leftover with nothing behind it. The AWS house is ~15 x 12 m, so a
            # 3.5 m horizon cannot see across a single room: rays into open space come
            # back NaN, and a NaN ray traces nothing, so a removed obstacle can never
            # be cleared (measured: 217/217 NaN straight ahead with nothing in range).
            # The OAK-D Lite reads well past this; the sim camera has no limit at all.
            # Keep in step with rover_rtabmap_rgbd's grid_range_max.
            'range_max': 4.5,
            'scan_height': 20,     # band of rows about the image centre
            'scan_time': 0.033,
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/camera_info'),
            ('scan', '/scan'),
        ],
    )

    # Build GAZEBO_MODEL_PATH: ugv_gazebo models + ugv_description share (for the
    # rover meshes) + AWS house models + whatever is already on the path.
    gazebo_model_path = [
        os.path.join(ugv_gazebo_share, 'models'), ':',
        os.path.dirname(ugv_description_share), ':',   # parent 'share' dir -> resolves model://ugv_description
        aws_small_house_path, '/models:',
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    ]

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
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
            description='Absolute path to aws-robomaker-small-house-world (supplies '
                        'the furniture meshes on GAZEBO_MODEL_PATH)'
        ),

        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('semantic_nav_bringup'),
                'worlds', 'small_house_semantic.world'
            ),
            description='Gazebo world file (default: baked semantic scenario '
                        'with the closed-door walls at door:119) — same default as '
                        'aws_small_house_tb3.launch.py so rover and TB3 runs are '
                        'comparable. Pass the stock AWS small_house.world to opt out.'
        ),

        DeclareLaunchArgument(
            'depth_only', default_value='true',
            description='DEFAULT. Sense like the REAL rover: strip the 2D LiDAR and '
                        'synthesise /scan from the depth camera '
                        '(depthimage_to_laserscan). Collapses the horizontal FOV from '
                        '360 to 59 degrees, matching the OAK-D Lite the hardware '
                        'actually carries. Set false to restore the simulated LiDAR, '
                        'which the real rover does not have.'
        ),

        SetEnvironmentVariable('UGV_MODEL', 'ugv_rover'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_ugv_rover,
        camera_scan_frame,
        depth_to_scan,
    ])
