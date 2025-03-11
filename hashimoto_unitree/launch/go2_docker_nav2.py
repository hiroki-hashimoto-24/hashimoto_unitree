import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    fast_lio_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('conory1_fast_lio'),
            'launch/'),'localization.launch.py']),
        launch_arguments={
            "map": "/home/unitree/colcon_ws/src/hashimoto_unitree/glim_map/glim_map.pcd",
            "use_rviz": "false",
        }.items()
    )

    conory1_navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('conory1_navigation2'),
            'launch/'),'conory1_navigation.launch.py']),
        launch_arguments={
            "map": "/home/unitree/colcon_ws/src/hashimoto_unitree/glim_map/map.yaml",
            "use_rviz": "false",
        }.items()
    )

    nav2_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hashimoto_unitree'),
            'launch/'),'nav2_rviz.launch.py']),
    ) 

    ld = LaunchDescription()
    ld.add_action(fast_lio_localization_launch)
    ld.add_action(conory1_navigation2_launch)
    ld.add_action(nav2_rviz_launch)

    return ld
