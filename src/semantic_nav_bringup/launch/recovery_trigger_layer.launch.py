from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    plan_topic = LaunchConfiguration('plan_topic')
    costmap_topic = LaunchConfiguration('costmap_topic')
    recovery_trigger_topic = LaunchConfiguration('recovery_trigger_topic')
    occupied_threshold = LaunchConfiguration('occupied_threshold')
    sample_radius_m = LaunchConfiguration('sample_radius_m')
    debounce_sec = LaunchConfiguration('debounce_sec')

    return LaunchDescription([
        DeclareLaunchArgument('plan_topic', default_value='/plan'),
        DeclareLaunchArgument('costmap_topic', default_value='/global_costmap/costmap'),
        DeclareLaunchArgument('recovery_trigger_topic', default_value='/recovery_trigger'),
        DeclareLaunchArgument('occupied_threshold', default_value='90'),
        DeclareLaunchArgument('sample_radius_m', default_value='0.0'),
        DeclareLaunchArgument('debounce_sec', default_value='1.0'),

        Node(
            package='semantic_nav_path_monitor',
            executable='plan_intersection_monitor',
            name='plan_intersection_monitor',
            output='screen',
            parameters=[{
                'plan_topic': plan_topic,
                'costmap_topic': costmap_topic,
                'recovery_trigger_topic': recovery_trigger_topic,
                'occupied_threshold': occupied_threshold,
                'sample_radius_m': sample_radius_m,
                'debounce_sec': debounce_sec,
            }],
        ),
    ])
