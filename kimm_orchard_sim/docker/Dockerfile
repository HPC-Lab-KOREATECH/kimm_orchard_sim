FROM althack/ros2:humble-cuda-gazebo-nvidia-2023-11-05

#nvidia-container-runtime 
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all} 
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics 

USER root 

# Set the timezone
ENV TZ Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Setup keys and sources.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 && \
    . /etc/os-release && \
    echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-latest.list

# Install gazebo packages
RUN apt-get update && apt-get install -y --no-install-recommends gazebo && rm -rf /var/lib/apt/lists/*

# Setup environment
EXPOSE 11345

# Setup ROS 2 dependencies
ARG ROS_DISTRO=humble

# Prevent console from interacting with the user
ARG DEBIAN_FRONTEND=noninteractive

# Prevent hash mismatch error for apt-get update
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# Set folder for RUNTIME_DIR
RUN mkdir tmp/runtime-root && chmod 0700 tmp/runtime-root
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

# Install non-Python/ROS dependencies
RUN apt-get install --no-install-recommends -yqqq \
    python3-pip

# Copy and run install.sh
COPY install.sh /tmp/install.sh
RUN chmod +x /tmp/install.sh

# Set up ROS 2 workspace
ARG WORKSPACE=/root/ros2_ws
ENV WORKSPACE=$WORKSPACE
WORKDIR $WORKSPACE

# Install ROS 2 packages and dependencies
RUN add-apt-repository ppa:borglab/gtsam-release-4.1 && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
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
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    pip install -U colcon-common-extensions

# Set ROS2 environment variables in /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=34" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> /root/.bashrc && \
    echo "echo 'ROS2 environment set, Domain = 34'" >> /root/.bashrc

# Set ROS2 Humble and ROS aliases in /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source ~/ros2_ws/install/local_setup.bash" >> /root/.bashrc && \
    echo "alias cw='cd ~/ros2_ws'" >> /root/.bashrc && \
    echo "alias cs='cd ~/ros2_ws/src'" >> /root/.bashrc && \
    echo "alias cb='cd ~/ros2_ws && colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cbp='cd ~/ros2_ws && colcon build --symlink-install --packages-select'" >> /root/.bashrc && \
    echo "alias sb='source ~/.bashrc'" >> /root/.bashrc && \
    echo "alias gb='gedit ~/.bashrc'" >> /root/.bashrc && \
    echo "alias rt='ros2 topic list'" >> /root/.bashrc && \
    echo "alias re='ros2 topic echo'" >> /root/.bashrc && \
    echo "alias rn='ros2 node list'" >> /root/.bashrc && \
    echo "alias testpub='ros2 run demo_nodes_cpp talker'" >> /root/.bashrc && \
    echo "alias testsub='ros2 run demo_nodes_cpp listener'" >> /root/.bashrc

CMD ["gzserver", "my-gazebo-app-args"]
