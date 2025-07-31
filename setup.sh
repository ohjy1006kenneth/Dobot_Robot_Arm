#!/bin/bash

source /opt/ros/humble/setup.sh
source install/local_setup.sh

sudo -v  # Ask for sudo password once

export IP_address=192.168.201.1
export DOBOT_TYPE=cr20

sudo bash ./src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash 
