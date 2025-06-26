#!/bin/bash

docker run --rm \
    --runtime nvidia \
    --gpus all \
    --ipc=host \
    --network=host \
    -v /dev:/dev \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v /home/diane/.Xauthority:/root/.Xauthority \
    -e XAUTHORITY=/root/.Xauthority \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=30 \
    -w /root/ros2_ws \
    --name shm2humble shm2ros:deploy /bin/bash -c 'source install/local_setup.bash && ros2 launch shm2ros shm2ros.launch.py'