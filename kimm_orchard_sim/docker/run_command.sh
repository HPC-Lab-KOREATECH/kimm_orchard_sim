#!/bin/bash
set -e

docker build -t kimm:v1.0 .

docker run -it \
    --privileged \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /home/$USER/docker_mounted/:/root/mounted_folder/ \
    -v="/tmp/.gazebo/:/root/.gazebo/" \
    --hostname $USER \
    --network host \
    --gpus all \
    --ipc=host \
    -P \
    --name kimm_orchard \
    kimm:v1.0 bash
