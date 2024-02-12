# kimm_orchard_sim
kimm_orchard_sim (2023)

# Dependencies
```
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

```
cd ~/ros2_ws/src
git clone <package>
cd ~/ros2_ws/src/kimm_orchard_sim/map/urdf
xacro orchard_geometry.urdf.xacro  > orchard_geometry.urdf
cd ~/ros2_ws && colcon build --symlink-install
```
# Docker setup

```
cd ~/ros2_ws/src/docker
./run_command.sh
```

# gazebo simulation 실행
```
ros2 launch kimm_orchard_sim gazebo.launch.py
```

# joint_state_publisher_gui 실행
```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

# tf pdf 생성
```
ros2 run tf2_tools view_frames
```

# Ranger Control
```
ros2 run kimm_orchard_sim keyboard_teleop.py
```
### Keyboard

1. **In-Phase mode** 

**w/W**(straight), **s/S**(back), **QAZCDE**(Other Phase) of 8 direction

2. **Opposite phase**

**q**(turn left && straight), **e**(turn right && straight), **z, c**

3. **Pivot turn**

**a or d** (Rotate in place)
