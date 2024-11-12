import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    imu_publisher_node = Node(
        package = "hashimoto_unitree",
        executable = "imu_publisher",
        output='screen'
    )

    # cmd2sport_node = Node(
    #     package = "hashimoto_unitree",
    #     executable = "cmd2sport",
    #     output='screen'
    # )

    cmd2sport_stop_node = Node(
        package = "hashimoto_unitree",
        executable = "cmd2sport_stop",
        output='screen'
    )

    # cmd2climb_node = Node(
    #     package = "hashimoto_unitree",
    #     executable = "cmd2climb",
    #     output='screen'
    # )
    
    msg_MID360_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch_ROS2/'),'msg_MID360_launch.py'])
    )

    ld = LaunchDescription()
    ld.add_action(imu_publisher_node)
    #ld.add_action(cmd2sport_node)
    ld.add_action(cmd2sport_stop_node)
    #ld.add_action(cmd2climb_node)
    ld.add_action(msg_MID360_cmd)

    return ld
