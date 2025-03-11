#!/bin/bash -e
# 端末1: lidar_imu_cmd2sport.launch.py を実行
gnome-terminal -- bash -c "ros2 launch hashimoto_unitree lidar_imu_cmd2sport.launch.py" &

sleep 1

# 端末2: ROS2コンテナを起動
echo "shell start"
gnome-terminal -- bash -c "\
    docker run -it --rm \
        --name ros2_humble \
        --network=host \
        --ipc=host \
        --mount type=bind,src=/home/unitree/docker_dir/share,dst=/home/unitree \
        -e DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/.Xauthority:/home/unitree/.Xauthority \
        -u unitree \
        ros2_humble;" &

sleep 3  # コンテナ起動待機 

gnome-terminal -- bash -c "docker exec -u unitree -i -t ros2_humble bash"
