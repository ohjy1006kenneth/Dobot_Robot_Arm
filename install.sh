sudo apt update
sudo apt-get update
sudo apt install ros-humble-joint-state-broadcaster
sudo apt install ros-humble-ros2controlcli
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-moveit

# ugv_sdk
sudo apt-get install build-essential git cmake libasio-dev

# Femto Mega
sudo apt install libgflags-dev nlohmann-json3-dev  \
ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev
