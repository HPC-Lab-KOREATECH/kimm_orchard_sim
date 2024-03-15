# kimm_orchard_sim

kimm_orchard_sim (2023)

![Screenshot from 2024-03-15 17-01-03](https://github.com/HPC-Lab-KOREATECH/kimm_orchard_sim/assets/157468651/b6549de4-58db-42f6-9e48-996b5e12e011)
![Screenshot from 2024-03-15 17-01-20](https://github.com/HPC-Lab-KOREATECH/kimm_orchard_sim/assets/157468651/76202934-4f10-474b-aa93-5a92c31663f3)


# Dependencies

```bash
rosdep update

cd ~/ros2_ws

rosdep install --from-paths src --ignore-src -r -y

sudo add-apt-repository ppa:borglab/gtsam-release-4.1 && \
sudo apt-get update && \
sudo apt-get upgrade -y && \
sudo apt-get install -y \
    nlohmann-json3-dev \
    libgtsam-dev \
    libgtsam-unstable-dev \
    ros-humble-gazebo-ros-pkgs \
    python3-rosdep2 \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rqt-robot-steering \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-gazebo-* \
    ros-humble-velodyne-gazebo-plugins \
    ros-humble-perception-pcl \
    ros-humble-pcl-msgs \
    ros-humble-vision-opencv \
    ros-humble-xacro \
    ros-humble-pcl-conversions \
    python3-pip \
    terminator \
    gedit \
    psmisc \
    cmake \
    libx11-dev \
    xorg-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    libglew-dev \
    libglfw3-dev && \

pip install transforms3d utm && \
sudo apt-get autoremove -y && \
sudo apt-get clean && \
sudo rm -rf /var/lib/apt/lists/* && \
pip install -U colcon-common-extensions
```

# First setup

```bash
cd ~/ros2_ws/src
git clone <package>
cd ~/ros2_ws/src/kimm_orchard_sim/map/urdf
xacro orchard_geometry.urdf.xacro  > orchard_geometry.urdf
cd ~/ros2_ws && colcon build --symlink-install
```

# Docker setup

```bash
cd ~/ros2_ws/src/docker
./run_command.sh
```

# gazebo simulation 실행

```bash
ros2 launch kimm_orchard_sim gazebo.launch.py
```
if you launch simulation with rviz
```bash
ros2 launch kimm_orchard_sim gazebo.launch.py rviz:=true
```

# tf pdf 생성

```bash
ros2 run tf2_tools view_frames
```

# Ranger Control

```bash
# Local path publisher 현재 실행된 global path 중 선택해서 가능 /gym, /straight, /u_turn, /circle
python3 /root/ros2_ws/src/kimm_orchard_sim/kimm_orchard_sim/scripts/path_debug/local_path_publisher.py /gym

# 제어 노드 실행 (Local path follower)
ros2 run control car_control
```

# LIO-SAM

```bash
ros2 launch lio_sam run.launch.py
```