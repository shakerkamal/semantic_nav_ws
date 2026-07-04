from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    resolve_service_arg = DeclareLaunchArgument(
        'resolve_service',
        default_value='/resolve_location',
        description='Resolve location service name'
    )

    execute_action_arg = DeclareLaunchArgument(
        'execute_action',
        default_value='/execute_pose',
        description='Execute pose action name'
    )

    semantic_node = Node(
        package='semantic_nav_semantics',
        executable='resolver_node',
        name='semantic_resolver',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    validator_node = Node(
        package='semantic_nav_validator',
        executable='validator_node',
        name='semantic_nav_validator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    executor_node = Node(
        package='semantic_nav_executor',
        executable='executor_node',
        name='semantic_nav_executor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    local_object_query_node = Node(
        package='semantic_nav_semantics',
        executable='local_object_query_node',
        name='local_object_query_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    door_state_monitor_node = Node(
        package='semantic_nav_semantics',
        executable='door_state_monitor_node',
        name='door_state_monitor_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        resolve_service_arg,
        execute_action_arg,
        semantic_node,
        validator_node,
        executor_node,
        local_object_query_node,
        door_state_monitor_node,
    ])
