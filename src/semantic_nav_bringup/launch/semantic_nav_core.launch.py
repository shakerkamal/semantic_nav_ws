import os

from ament_index_python.packages import get_package_share_directory

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

    # Default stays map_v001.json so the TB3 stack is unchanged. The ugv_rover
    # overrides this to map_v002.json (its world has 8 fewer furniture models),
    # so BOTH the resolver and local_object_query read the same map as the LLM and
    # the orchestrator — otherwise the rover would resolve against objects that no
    # longer exist in its world.
    semantic_map_path_arg = DeclareLaunchArgument(
        'semantic_map_path',
        default_value=os.path.join(
            get_package_share_directory('semantic_nav_semantics'),
            'config',
            'map_v001.json',
        ),
        description='Absolute path to the object-centric semantic map.'
    )
    semantic_map_path = LaunchConfiguration('semantic_map_path')

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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'semantic_map_path': semantic_map_path,
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
            'map_path': semantic_map_path,   # note: this node's param is 'map_path'
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
        semantic_map_path_arg,
        resolve_service_arg,
        execute_action_arg,
        semantic_node,
        validator_node,
        executor_node,
        local_object_query_node,
        door_state_monitor_node,
    ])
