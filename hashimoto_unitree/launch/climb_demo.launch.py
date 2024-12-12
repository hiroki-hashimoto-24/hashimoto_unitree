import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    rviz_MID360_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch_ROS2/'),'rviz_MID360_launch.py'])
    )

    glim_ros_node = Node(
        package = "glim_ros",
        executable = "glim_rosnode",
        parameters = [{'config_path': "/home/hashimoto/colcon_ws/src/hashimoto_unitree/glim_config"},
                    {'dump_path': "/home/hashimoto/colcon_ws/src/hashimoto_unitree/glim_map"}],
        output='screen'
    )

    cmd2climb_node = Node(
        package = "hashimoto_unitree",
        executable = "cmd2climb_waypoint",
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(rviz_MID360_launch)
    ld.add_action(glim_ros_node)
    ld.add_action(cmd2climb_node)
    return ld