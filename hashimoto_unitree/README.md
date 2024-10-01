# hashimoto_unitree

unitreeに関する自作プログラム

# Install
```bash
cd <ros2_ws>/src
git clone https://github.com/hiroki-hashimoto-24/hashimoto_unitree.git
cd ..
colcon build

```

# usage

//mujoco
```bash
source ~/unitree_ros2/setup_local.sh
cd ~/unitree_mujoco/simulate/build
./unitree_mujoco

source ~/unitree_ros2/setup_local.sh
export ROS_DOMAIN_ID=0
ros2 run hashimoto_unitree mujoco_prog
```

//move
ros2 run hashimoto_unitree sport_prog
