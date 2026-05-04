from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    semantic_db_path_arg = DeclareLaunchArgument(
        'semantic_db_path',
        default_value='',
        description='Optional absolute path to semantic_db.json'
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
            'semantic_db_path': LaunchConfiguration('semantic_db_path')
        }]
    )

    validator_node = Node(
        package='semantic_nav_validator',
        executable='validator_node',
        name='semantic_nav_validator',
        output='screen',
    )

    executor_node = Node(
        package='semantic_nav_executor',
        executable='executor_node',
        name='semantic_nav_executor',
        output='screen'
    )

    return LaunchDescription([
        semantic_db_path_arg,
        resolve_service_arg,
        execute_action_arg,
        semantic_node,
        validator_node,
        executor_node
    ])