import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    lidar_imu_cmd2sport_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hashimoto_unitree'),
            'launch/'),'lidar_imu_cmd2sport.launch.py'])
    )

    fast_lio_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('conory1_fast_lio'),
            'launch/'),'localization.launch.py']),
        launch_arguments={
            "map": "/home/hashimoto/colcon_ws/src/hashimoto_unitree/fast_lio_map/fast_lio.pcd",
            "use_rviz": "false",
        }.items()
    )

    conory1_navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('conory1_navigation2'),
            'launch/'),'conory1_navigation.launch.py']),
        launch_arguments={
            "map": "/home/hashimoto/colcon_ws/src/hashimoto_unitree/fast_lio_map/map.yaml",
            "use_rviz": "true",
        }.items()
    )

    obstacle_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hashimoto_unitree'),
            'launch/'),'obstacle_detector.launch.py']),
        launch_arguments={
            "use_rviz": "false",
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(lidar_imu_cmd2sport_launch)
    ld.add_action(fast_lio_localization_launch)
    ld.add_action(conory1_navigation2_launch)
    ld.add_action(obstacle_detector_launch)

    return ld
