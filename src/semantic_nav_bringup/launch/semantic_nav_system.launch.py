import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rtabmap_demo = get_package_share_directory('rtabmap_demos')
    # Get the launch directory
    bringup_dir = get_package_share_directory('semantic_nav_bringup')

    # Declare launch arguments
    semantic_db_path_arg = DeclareLaunchArgument(
        'semantic_db_path',
        default_value='',
        description='Optional absolute path to semantic_db.json'
    )

    rtabmap_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rtabmap_demo, 
                'launch',
                'turtlebot3', 
                'turtlebot3_sim_rgbd_scan_demo.launch.py'
                )
            )
        )

    semantic_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir, 
                'launch', 
                'semantic_nav_core.launch.py'
                )
            ),
        launch_arguments={
            'semantic_db_path': LaunchConfiguration('semantic_db_path'),
            }.items()
        )

    return LaunchDescription([
        semantic_db_path_arg,
        rtabmap_demo,
        semantic_core
    ])