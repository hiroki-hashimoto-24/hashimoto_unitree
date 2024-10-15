from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    navigation_node = Node(
        package = "hashimoto_unitree",
        executable = "waypoint_pot_goal",
        parameters=[{'mode': LaunchConfiguration('mode')},
                    {'file_name': LaunchConfiguration('file_name')},
                    {'speed': LaunchConfiguration('speed')}],
        remappings=[('/odometry', '/glim_ros/odom')],
        output = "screen",
    )
    cmd2sport_node = Node(
        package = "my_unitree",
        executable = "cmd2sport",
    )

    ld = LaunchDescription()
    ld.add_action(navigation_node)
    ld.add_action(cmd2sport_node)

    return ld
