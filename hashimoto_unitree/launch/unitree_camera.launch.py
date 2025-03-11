import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Package path
    this_directory = get_package_share_directory('hashimoto_unitree')
    rviz_file = os.path.join(this_directory, 'rviz', 'unitree_camera.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='screen'
    )

    camera_node = Node(
        package = "gscam2",
        executable = "gscam_main",
        parameters=[{'gscam_config':'udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! queue !  application/x-rtp, media=video, width=1280, height=720, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'},
                    {'camera_info_url':'file:///home/unitree/colcon_ws/src/hashimoto_unitree/camera_info/camera_info.yaml'}],
        output='screen'
    )

    image_proc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('image_proc'),
            'launch/'),'image_proc.launch.py'])
    )

    ld = LaunchDescription()
    ld.add_action(rviz_node)
    ld.add_action(camera_node)
    ld.add_action(image_proc_launch)

    return ld
