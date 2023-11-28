#!/bin/bash
set -e  # Exit on error

# Install ROS 2 packages and dependencies
apt-get update
apt-get upgrade -y

apt-get install -y \
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
    libglfw3-dev

# Install additional Python package using pip
pip install transforms3d

apt-get update
apt-get upgrade -y