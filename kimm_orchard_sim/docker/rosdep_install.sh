#!/bin/bash
set -e  # Exit on error

# Install ROS 2 packages and dependencies
apt-get update
apt-get upgrade -y

cd ~/ros2_ws
rosdep update
rosdep install -i --from-paths src --ignore-src --rosdistro humble -y --skip-keys pcl_1.10 --skip-keys Eigen --skip-keys GTSAM --skip-keys PCL --skip-keys OpenCV
