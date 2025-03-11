import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Package path
    this_directory = get_package_share_directory('hashimoto_unitree')

    rviz_file = os.path.join(this_directory, 'rviz', 'nav2_view_modified.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])
