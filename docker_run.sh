#!/bin/bash

docker run --rm -it \
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
    -v ./ros2_ws:/root/ros2_ws \
    -w /root/ros2_ws \
    --name shm2humble shm2ros:humble /bin/bash