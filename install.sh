#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# install.sh
#   One-shot environment + dependency setup for the Dobot_Robot_Arm workspace.
#   Run once after `git clone`. Requires ROS 2 Humble already installed.
# -----------------------------------------------------------------------------
set -e

# Resolve repo root regardless of where this script is invoked from.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[install.sh] APT update"
sudo apt update

echo "[install.sh] ROS 2 Humble packages"
sudo apt install -y \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-joint-state-broadcaster \
    ros-humble-ros2controlcli \
    ros-humble-ros2-controllers \
    ros-humble-moveit \
    ros-humble-nav2-bringup \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-state-publisher \
    ros-humble-image-transport

echo "[install.sh] System libs (ugv_sdk, can-utils, KSJ profiler deps)"
sudo apt install -y \
    build-essential git cmake libasio-dev \
    can-utils \
    libgflags-dev nlohmann-json3-dev libdw-dev

echo "[install.sh] Optional: Orbbec Femto Mega driver deps"
sudo apt install -y \
    ros-"${ROS_DISTRO}"-image-transport-plugins \
    ros-"${ROS_DISTRO}"-compressed-image-transport \
    ros-"${ROS_DISTRO}"-image-publisher \
    ros-"${ROS_DISTRO}"-camera-info-manager \
    ros-"${ROS_DISTRO}"-diagnostic-updater \
    ros-"${ROS_DISTRO}"-diagnostic-msgs \
    ros-"${ROS_DISTRO}"-statistics-msgs \
    ros-"${ROS_DISTRO}"-backward-ros

echo "[install.sh] Python packages"
pip install --user opencv-python "numpy<2"

echo "[install.sh] Fetching third-party ROS 2 packages via vcstool"
cd "${REPO_ROOT}"
vcs import src < dependencies.repos

echo "[install.sh] Done. Next steps:"
echo "    cd ${REPO_ROOT}"
echo "    colcon build --symlink-install"
echo "    source install/setup.bash"
