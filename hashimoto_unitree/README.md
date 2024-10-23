# hashimoto_unitree

unitreeに関する自作プログラム

# Dependencies
unitree_sdk2  
unitree_mujoco  
unitree_ros2  
GLIM  

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

waypoint navigation  
ファイル読み込みのパスがプログラム直書きなので適宜修正することで使用可能  
```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~colcon_ws/src/git-hashimoto/hashimoto_unitree/glim_config)
ros2 launch hashimoto_unitree waypoint.launch.py
```

PC引き継ぎのテスト
