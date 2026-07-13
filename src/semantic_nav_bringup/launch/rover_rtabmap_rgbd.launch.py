"""RTAB-Map RGB-D SLAM for the ugv_rover, with a navigation-grade occupancy grid.

Replaces ugv_gazebo/launch/slam/rtabmap_rgbd.launch.py, whose parameter dict is
hardcoded and cannot be overridden from an including launch file. Same node,
same topics, same frames — the difference is the Grid/* block below.

WHY THIS EXISTS — the Waveshare defaults smear the map on every spin:

  Grid/RayTracing = false  free space is never carved, so once a cell is marked
                           occupied nothing can ever clear it again. Obstacle
                           cells only accumulate.
  Grid/3D         = true   a 3D grid projected down, not a 2D nav grid.
  Grid/RangeMax   = 0      unlimited insertion range.

The grid itself was ALREADY built from the laser scan, not the depth cloud:
rtabmap auto-sets Grid/Sensor=0 whenever subscribe_scan is true. So the range
was in practice bounded by the LiDAR's own 3.5 m anyway. The one setting that
genuinely cannot self-correct is Grid/RayTracing=false — a grid that can mark
but never clear will only ever get denser.

Registration (Reg/*) is left EXACTLY as Waveshare had it: map->odom was measured
stable through a 14 s spin with zero loop closures, so the pose graph was never
the problem and is not what we are changing here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    queue_size = LaunchConfiguration('queue_size')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    grid_range_max = LaunchConfiguration('grid_range_max')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'queue_size': queue_size,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': True,
        'use_action_for_goal': True,
        'qos_image': qos,
        'qos_imu': qos,

        # --- Registration: unchanged from Waveshare (verified healthy). ---
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySigma': '0',   # IMU constraints off; we are already 2D

        # --- Occupancy grid: the fix. See module docstring. ---
        # rtabmap declares EVERY Grid/* param as a string. A bare LaunchConfiguration
        # here gets auto-converted to a double by launch_ros, and rtabmap then aborts
        # with SIGABRT before printing anything ("is of type {string}, setting it to
        # {double} is not allowed"). ParameterValue pins it back to str.
        # Grid/Sensor=0 -> grid from the laser scan. rtabmap already infers this
        # because subscribe_scan is true; stating it explicitly silences its warning
        # and keeps the intent visible. (Grid/FromDepth is the OLD name and is
        # silently ignored by this build — don't use it.) In depth_only mode /scan
        # comes from depthimage_to_laserscan, so this still holds.
        'Grid/Sensor': '0',
        'Grid/3D': 'false',              # 2D nav grid, not a projected 3D one
        'Grid/RayTracing': 'true',       # carve free space so stale cells CAN clear
        'Grid/RangeMax': ParameterValue(grid_range_max, value_type=str),
    }

    remappings = [
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
    ]

    rtabmap_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d'],   # delete the previous DB (~/.ros/rtabmap.db)
    )

    rtabmap_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[
            parameters,
            {'Mem/IncrementalMemory': 'False', 'Mem/InitWMWithAllNodes': 'True'},
        ],
        remappings=remappings,
    )

    # Waveshare launches this whenever use_rviz is false, i.e. always in our
    # stack — a heavy GUI nobody was looking at. Opt-in here.
    rtabmap_viz_node = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[parameters],
        remappings=remappings,
        condition=IfCondition(rtabmap_viz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('queue_size', default_value='20'),
        DeclareLaunchArgument('qos', default_value='2'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Run RTAB-Map in localization mode instead of SLAM.'),
        DeclareLaunchArgument(
            'grid_range_max', default_value='3.5',
            description='Max range (m) of observations inserted into the occupancy '
                        'grid. Matches the rover LiDAR (model.sdf <max>3.5</max>) and '
                        'the depth_only scan. Waveshare left it 0 = unlimited.'),
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true',
            description='Launch the rtabmap_viz GUI (loop closures, feature matches, '
                        'the pose graph). Set false for headless/batch runs.'),

        rtabmap_slam,
        rtabmap_localization,
        rtabmap_viz_node,
    ])
