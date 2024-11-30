#!/bin/bash

i=0
for (( ; ; ))
do
    i=$((i+1))
    echo $i
    ros2 topic pub --once /odom nav_msgs/msg/Odometry "{pose: {pose: {position: {x: $i}}}}"
    sleep 0.2
done
