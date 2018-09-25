#!/bin/bash

sudo bash -c "
source /opt/ros/kinetic/setup.bash; 
source /home/nvidia/catkin_ws/devel/setup.bash; 
roslaunch ros_mpu6050_node mpu6050.launch 
"

