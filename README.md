# kimm_orchard_sim
kimm_orchard_sim (2023)

# Dependencies
```
sudo apt-get install ros-humble-gazebo-ros-pkgs

sudo apt install python3-rosdep2

rosdep update

cd ~/ros2_ws

rosdep install --from-paths src --ignore-src -r -y

sudo apt-get install ros-humble-ros2-control

sudo apt install ros-humble-ros2-controllers

sudo apt-get install ros-humble-joint-state-publisher-gui

sudo apt-get install ros-humble-rqt-robot-steering

sudo apt install ros-humble-tf2-tools ros-humble-tf-transformations
```

# gazebo simulation 실행
```ros2 launch bcr_bot gazebo.launch.py```

# joint_state_publisher_gui 실행
```ros2 run joint_state_publisher_gui joint_state_publisher_gui```

# tf pdf 생성
```ros2 run tf2_tools view_frames```



