import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    msg_MID360_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch_ROS2/'),'msg_MID360_launch.py'])
    )
    
    imu_publisher_node = Node(
        package = "hashimoto_unitree",
        executable = "imu_publisher",
        output='screen'
    )

    cmd2sport_node = Node(
        package = "hashimoto_unitree",
        #executable = "cmd2sport",
        executable = "cmd2sport_stop",
		#executable = "cmd2climb_down",
        output='screen'
    )
    
    lidar_stop_node = Node(
        package = "hashimoto_unitree",
        executable = "lidar_stop",
        #remappings=[('livox/lidar', 'utlidar/cloud')],
        parameters = [{'axis_x': 0.50},
                    {'axis_y': 0.20},
                    {'center_x': -0.1}],
        output='screen'
    )

	height_map_node = Node(
        package = "hashimoto_unitree",
        executable = "height_map_sh_width",
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(msg_MID360_cmd)
    ld.add_action(imu_publisher_node)
    ld.add_action(cmd2sport_node)
    ld.add_action(lidar_stop_node)
	#ld.add_action(height_map_node)
    
    return ld
