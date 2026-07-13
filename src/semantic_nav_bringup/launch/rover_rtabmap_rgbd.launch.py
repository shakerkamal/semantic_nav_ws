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
    optimizer_robust = LaunchConfiguration('optimizer_robust')
    map_always_update = LaunchConfiguration('map_always_update')

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

        # --- Registration ---
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySigma': '0',   # IMU constraints off; we are already 2D

        # Robust (Vertigo switchable-constraint) graph optimization. Waveshare leaves
        # this false, which means a SINGLE false loop closure drags the whole graph.
        # Measured on a depth_only run: 36 global loop closures, 35 of them excellent
        # (median 1.0 cm from ground truth) and ONE at node 266->187 off by 299.7 cm /
        # 10.4 deg. That one link warped map->odom to (0.33, -2.36 m, 14.7 deg) and
        # produced the overlapping, rotated map. Robust optimization down-weights such
        # an outlier instead of believing it.
        #
        # This matters far more with depth_only: a 59 deg FOV sees less of each place,
        # so visual bag-of-words mismatches between near-identical rooms are likelier.
        # Needs Optimizer/Strategy 1 (g2o) or 2 (GTSAM) — this build is 2, GTSAM linked.
        # ParameterValue(str) for the same reason as Grid/RangeMax below: a bare
        # LaunchConfiguration gets auto-converted (here to bool) and rtabmap aborts.
        'Optimizer/Robust': ParameterValue(optimizer_robust, value_type=str),

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

        # With the rtabmap default (false) the occupancy grid is ONLY refreshed when a
        # new node joins the graph, and a node needs RGBD/LinearUpdate (0.1 m) or
        # RGBD/AngularUpdate (0.1 rad) of MOTION. A stationary robot therefore never
        # updates its map, however long it stares at a doorway that has just opened.
        # That is the real reason the recovery had to spin: the spin was not a
        # perception strategy, it was a trick to force node creation.
        #
        # true = update the grid from the latest sensor data at the current odom pose,
        # no motion required. With Grid/RayTracing this lets the rover clear a freed
        # doorway just by FACING it. Costs some CPU, and the update leans on odom
        # rather than an optimised node pose.
        'map_always_update': map_always_update,
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
            'grid_range_max', default_value='4.5',
            description='Max range (m) of observations inserted into the occupancy '
                        'grid; kept in step with depthimage_to_laserscan range_max. '
                        'Was 3.5 (the Waveshare LiDAR ceiling) — far too short for a '
                        '15 x 12 m house, and the reason obstacles could not clear: '
                        'rays past 3.5 m come back NaN and trace nothing. Waveshare '
                        'left it 0 = unlimited, which is the other extreme.'),
        DeclareLaunchArgument(
            'map_always_update', default_value='true',
            description='Refresh the occupancy grid even when the robot is not moving. '
                        'rtabmap defaults to false, which means the map only updates on '
                        'new graph nodes (>=0.1 m / >=0.1 rad of motion) — so a parked '
                        'robot can never see a doorway clear, which is why recovery had '
                        'to spin. Set false to restore the stock behaviour.'),
        DeclareLaunchArgument(
            'optimizer_robust', default_value='true',
            description='Robust (Vertigo) graph optimization: reject outlier loop '
                        'closures instead of letting one warp the map. Set false to '
                        'reproduce the Waveshare behaviour.'),
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true',
            description='Launch the rtabmap_viz GUI (loop closures, feature matches, '
                        'the pose graph). Set false for headless/batch runs.'),

        rtabmap_slam,
        rtabmap_localization,
        rtabmap_viz_node,
    ])
