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
    rviz_file = os.path.join(this_directory, 'rviz', 'obstacle_detector.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz', default_value='false',
        description='Use rviz to show result if true'
    )

    obstacle_extractor_node = Node(
        package = "obstacle_detector",
        executable = "obstacle_extractor_node",
        #remappings = [('/scan', '/scan')],
        #remappings = [('/pcl', '/scan_filtered')],
        #remappings = [('/pcl2', '/livox/lidar')],
        remappings = [('/pcl2', '/cloud_registered_body')],
        parameters=[{'active': True},
                    {'use_scan': False},
                    {'use_pcl': False},
                    {'use_pcl2': True},
                    {'use_split_and_merge': True},
                    {'circles_from_visibles': True},
                    {'discard_converted_segments': True},
                    {'transform_coordinates': True},
                    {'min_group_points': 10},
                    {'max_group_distance': 0.1},
                    {'distance_proportion': 0.00628},
                    {'max_split_distance': 0.2},
                    {'max_merge_separation': 0.2},
                    {'max_merge_spread': 0.2},
                    {'max_circle_radius': 0.6},
                    {'radius_enlargement': 0.3},
                    #{'frame_id': 'livox_frame'},
                    {'frame_id': 'base_link'}],
        output='screen'
    )

    obstacle_tracker_node = Node(
        package = "obstacle_detector",
        executable = "obstacle_tracker_node",
        #remappings = [('/odom', '/odometry/filtered')],
        #remappings = [('/tracked_obstacles', '/obstacles')],
        #remappings = [('/odtracked_obstacles_visualizationom', '/obstacles_visualization')],
        parameters = [{'active': True},
                    {'loop_rate': 100.0},
                    {'tracking_duration': 0.3},
                    {'min_correspondence_cost': 0.6},
                    {'std_correspondence_dev': 0.15},
                    {'process_variance': 0.1},
                    {'process_rate_variance': 0.1},
                    {'measurement_variance': 1.0},
                    #{'frame_id': 'livox_frame'},
                    {'frame_id': 'base_link'}],
        output='screen'
    )

    obstacle_hit_node = Node(
        package = "hashimoto_unitree",
        executable = "obstacle_hit",
        parameters = [{'segment_axis_x': 0.6},
                    {'segment_axis_y': 0.3},
                    {'circle_axis_x': 0.6},
                    {'circle_axis_y': 0.3},
                    {'detect_object': 2.0},
                    {'easy_log': True}],
        output = 'screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(obstacle_extractor_node)
    ld.add_action(obstacle_tracker_node)
    ld.add_action(obstacle_hit_node)
    ld.add_action(rviz_node)
    return ld
