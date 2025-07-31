#!/bin/bash

source /opt/ros/humble/setup.sh
source install/local_setup.sh

sudo -v  # Ask for sudo password once

export IP_address=192.168.201.1
export DOBOT_TYPE=cr20

sudo bash ./src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash 

# gnome-terminal --tab -- bash -c "ros2 launch orbbec_camera femto_mega.launch.py; exec bash"
# read -p "Started orbbec_camera. Press Enter to continue..."

# gnome-terminal --tab -- bash -c "sudo bash ./src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash; exec bash"
# read -p "Started CAN2USB. Press Enter to continue..."

# gnome-terminal --tab -- bash -c "ros2 launch ranger_bringup ranger.launch.xml; exec bash"
# read -p "Started ranger. Press Enter to continue..."

# gnome-terminal --tab -- bash -c "ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py; exec bash"
# read -p "Started dobot bringup. Press Enter to continue..."

# gnome-terminal --tab -- bash -c "ros2 service call /dobot_bringup_ros2/srv/EnableRobot dobot_msgs_v4/srv/EnableRobot '{enable: true}'; exec bash"
# read -p "Enabled dobot. Press Enter to continue..."

# gnome-terminal --tab -- bash -c "ros2 run image_processing line_follower; exec bash"
# read -p "Started line follower. Press Enter to continue..."

# gnome-terminal --tab -- bash -c "ros2 run ranger_control line_follower_controller; exec bash"
# read -p "Started controller. Press Enter to continue..."

# gnome-terminal --tab -- bash -c "ros2 run dobot_arm script_runner; exec bash"
# echo "🎉 All nodes launched!"
