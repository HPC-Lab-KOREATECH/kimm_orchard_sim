set -e

docker pull althack/ros2:humble-cuda-gazebo-nvidia-2023-11-05

docker build -t kimm_orchard_gazebo:v1 .

docker run -it --privileged -e DISPLAY=$DISPLAY --env="QT_X11_NO_MITSHM=1" -v="/tmp/.gazebo/:/root/.gazebo/" -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v /home/hy/docker_mounted/kimm_orchard/:/root/mounted_folder/ --hostname hy-B660M-HD3P --network host --gpus all --name kimm_orchard_v1 kimm_orchard_gazebo:v1 bash