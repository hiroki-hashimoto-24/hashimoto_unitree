# hashimoto_unitree

unitreeに関する自作プログラム

# Dependencies
unitree_sdk2  
unitree_mujoco  
unitree_ros2  
GLIM  
fast_lio(パーソルテクノロジー版)  

# Install
```bash
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/hiroki-hashimoto-24/hashimoto_unitree.git
cd ..
colcon build

```

# usage

mujoco  
```bash
source ~/unitree_ros2/setup_local.sh
cd ~/unitree_mujoco/simulate/build
./unitree_mujoco

source ~/unitree_ros2/setup_local.sh
export ROS_DOMAIN_ID=0
ros2 run hashimoto_unitree mujoco_prog
```

high level command  
```bash
ros2 run hashimoto_unitree sport_prog
```

glim  
```bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~colcon_ws/src/git-hashimoto/hashimoto_unitree/glim_config)
```

fast_lio_localizationと経路作成  
```bash
ros2 launch conory1_navigation2 conory1_navigation.launch.py map:=/home/hashimoto/pointcloud_to_2dmap/map/map.yaml use_rviz:=true
ros2 run my_unitree goal_pose_editor --ros-args -p file_name:=sample1.csv
```

navigation2の利用  
```bash
ros2 launch hashimoto_unitree lidar_imu_cmd2sport.launch.py
ros2 launch conory1_fast_lio localization.launch.py map:=/home/hashimoto/ros2_ws/src/fast_lio/PCD/result.pcd use_rviz:=false
ros2 launch nav2_waypoint_publisher patrol_waypoints_publisher.launch.py waypoints:=/home/hashimoto/colcon_ws/src/my_unitree/nav2_waypoint/test3.csv number_of_loops:=0
```

waypoint navigation  
ファイル読み込みのパスがプログラム直書きなので適宜修正することで使用可能  
```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
ros2 launch hashimoto_unitree waypoint.launch.py
```
