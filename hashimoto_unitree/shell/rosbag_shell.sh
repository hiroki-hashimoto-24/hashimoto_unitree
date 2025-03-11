#!/bin/bash -e

gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 rviz_MID360_launch.py"

cd ~/bag_file
gnome-terminal -- bash -c "ros2 bag record /livox/imu /livox/lidar"
