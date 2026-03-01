#!/bin/bash

# Source ROS2 and all workspaces in the correct overlay order
source /opt/ros/humble/setup.sh
source /home/cats/ws_livox/install/setup.sh
source /home/cats/Dobot_Robot_Arm/install/setup.sh

sudo -v  # Ask for sudo password once

export IP_address=192.168.201.1
export DOBOT_TYPE=cr20

sudo bash ./src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash 

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PWD/src/laser_scanner/include/KSJApi.bin/x64

