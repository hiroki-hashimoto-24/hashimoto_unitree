FROM ros:humble-ros-core

# ホストでGUIを表示するために必要
ENV QT_X11_NO_MITSHM=1

# unitreeユーザ環境の設定
# コンテナ内の/home/unitreeと、ホストの適当なディレクトリをバインドすると扱いやすい
RUN /sbin/groupadd -g 1000 unitree
RUN /sbin/useradd -u 1000 -g 1000 -d /home/unitree -s /bin/bash unitree 
RUN /bin/mkdir -p /home/unitree
RUN /bin/chown 1000:1000 /home/unitree

# パッケージの更新
RUN apt update
RUN apt dist-upgrade -y

RUN apt-get install -y python3-colcon-common-extensions
RUN apt install build-essential -y
RUN apt install nano -y
RUN apt-get install python3-pip -y
RUN apt install python3-pandas -y

RUN apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup -y
RUN apt install ros-humble-pcl-* -y

RUN apt install ros-humble-rmw-cyclonedds-cpp -y
RUN apt install ros-humble-rosidl-generator-dds-idl -y

RUN apt install ros-humble-image-pipeline -y
RUN apt install ros-humble-image-common -y

RUN apt install git -y

# GUI
RUN apt install qtbase5-dev qtchooser qt5-qmake qttools5-dev-tools -y
RUN apt install libx11-xcb1 libxcb-xkb1 libxcb-cursor0 libxcb-icccm4 libxcb-image0 -y
RUN apt install libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-shape0 -y
RUN apt install libxcb-xinerama0 libxcb-util1 libxcb-xinput0 libxkbcommon-x11-0 x11-utils -y

#COPY ./ros_entrypoint.sh /
#RUN chmod +x /ros_entrypoint.sh
#ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
