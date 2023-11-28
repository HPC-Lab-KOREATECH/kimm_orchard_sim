FROM althack/ros2:humble-cuda-gazebo-nvidia-2023-11-05

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

CMD ["gzserver", "my-gazebo-app-args"]
