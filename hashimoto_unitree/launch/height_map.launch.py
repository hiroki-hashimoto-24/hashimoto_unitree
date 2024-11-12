import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    this_directory = get_package_share_directory('hashimoto_unitree')
    rviz_file = os.path.join(this_directory, 'rviz', 'height_map.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz', default_value='false',
        description='Use rviz to show result if true'
    )

    height_map_node = Node(
        package = "hashimoto_unitree",
        executable = "height_map",
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(height_map_node)
    ld.add_action(rviz_node)
    return ld