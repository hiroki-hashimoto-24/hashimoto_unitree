# hashimoto_unitree

unitreeに関する自作プログラム

絶対パスで記述している箇所は適宜各個人のインストール場所(ユーザー名)に変更して使用してください  
特にlaunchファイルが絶対パス直書きの箇所が多いので変更する必要があります

# Dependencies
unitree_sdk2  
unitree_mujoco  
unitree_ros2  
livox_ros_driver2  
GLIM  
ply_to_pcd  
pointcloud_to_2dmap  
fast_lio(パーソルテクノロジー版)  
obstacle_detector  

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
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~colcon_ws/src/hashimoto_unitree/glim_config) -p dump_path:=$(realpath ~/colcon_ws/src/hashimoto_unitree/glim_map)
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```
```bash
ros2 run glim_ros offline_viewer
```

.ply->.pcd  
```bash
cd ~/ply_to_pcd/build
./ply_to_pcd ~/colcon_ws/src/hashimoto_unitree/glim_map/glim_map.ply
eog ~/colcon_ws/src/my_unitree/glim_map/map.png
```

fast_lio_localizationによる地図作成
```bash
ros2 launch conory1_fast_lio mapping.launch.py
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
```bash
ros2 service call /map_save fast_lio/srv/MapSave "filepath: $HOME/colcon_ws/src/hashimoto_unitree/fast_lio_map/fast_lio.pcd"
pcl_viewer result.pcd
```
```bash
cd ~/pointcloud_to_2dmap/build
./pointcloud_to_2dmap -w 400 -h 400 ~/colcon_ws/src/hashimoto_unitree/fast_lio_map/fast_lio.pcd ~/colcon_ws/src/hashimoto_unitree/fast_lio_map
eog ~/colcon_ws/src/hashimoto_unitree/fast_lio_map/map.png
```

navigation2と経路作成  
```bash
ros2 launch conory1_navigation2 conory1_navigation.launch.py map:=/home/unitree/colcon_ws/src/hashimoto_unitree/fast_lio_map/map.yaml use_rviz:=true
ros2 run hashimoto_unitree goal_pose_editor --ros-args -p file_name:=sample1.csv
```

navigation2の利用  
```bash
ros2 launch hashimoto_unitree lidar_imu_cmd2sport.launch.py
ros2 launch conory1_fast_lio localization.launch.py map:=/home/unitree/colcon_ws/src/hashimoto_unitree/fast_lio_map/fast_lio.pcd use_rviz:=false
ros2 launch conory1_navigation2 conory1_navigation.launch.py map:=/home/unitree/colcon_ws/src/hashimoto_unitree/fast_lio_map/map.yaml use_rviz:=true
ros2 launch nav2_waypoint_publisher patrol_waypoints_publisher.launch.py waypoints:=/home/unitree/colcon_ws/src/hashimoto_unitree/nav2_waypoint/root9.csv number_of_loops:=0
```

waypoint navigation(自作ナビゲーションパッケージ)  
ファイル読み込みのパスがプログラム直書きなので適宜修正することで使用可能  
```bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~colcon_ws/src/hashimoto_unitree/glim_config) -p dump_path:=$(realpath ~/colcon_ws/src/hashimoto_unitree/glim_map)
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
ros2 launch hashimoto_unitree waypoint.launch.py
```

waypoint navigation(自作ナビゲーションパッケージ):自己位置にfast_lioを使う場合
```bash
ros2 launch hashimoto_unitree lidar_imu_cmd2sport.launch.py
ros2 launch conory1_fast_lio mapping.launch.py
ros2 run hashimoto_unitree waypoint_pot_goal --ros-args -p mode:=2 -p file_name:=front1.csv -p speed:=0.2 --remap odometry:=fast_lio/odom
```

obstacle_detector  
```bash
ros2 launch hashimoto_unitree lidar_imu_cmd2sport.launch.py
ros2 launch hashimoto_unitree obstacle_detector.launch.py use_rviz:=true
```

自律移動のデモ  
```bash
ros2 launch hashimoto_unitree navigation2_demo.launch.py
ros2 launch nav2_waypoint_publisher patrol_waypoints_publisher.launch.py waypoints:=/home/unitree/colcon_ws/src/hashimoto_unitree/nav2_waypoint/root9.csv number_of_loops:=0
```

段差の登り降り(段差位置既知)のデモ  
```bash
ros2 launch hashimoto_unitree climb_demo.launch.py
ros2 launch hashimoto_unitree waypoint_climb.launch.py mode:=2 file_name:=root1.csv speed:=0.2
```

段差を見つけて登り降りする
```bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/colcon_ws/src/hashimoto_unitree/glim_config) -p dump_path:=$(realpath ~/colcon_ws/src/hashimoto_unitree/glim_map)
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
ros2 run hashimoto_unitree cmd2climb_down
ros2 launch hashimoto_unitree height_map.launch.py use_rviz:=true
ros2 run hashimoto_unitree waypoint_pot_goal --ros-args -p mode:=2 -p file_name:=front1.csv -p speed:=0.2 --remap odometry:=glim_ros/odom
```