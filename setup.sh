#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# setup.sh
#   Per-shell environment setup. Source this in every terminal that will run
#   the surface-reconstruction stack:
#       source ./setup.sh
# -----------------------------------------------------------------------------

# Resolve repo root regardless of where this script is sourced from.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 and the workspace overlay.
source /opt/ros/humble/setup.sh
if [ -f "${REPO_ROOT}/install/setup.sh" ]; then
    source "${REPO_ROOT}/install/setup.sh"
fi

# Cache sudo credentials once for the CAN bringup below.
sudo -v

# Dobot bringup parameters.
export IP_address=192.168.201.1
export DOBOT_TYPE=cr20

# Bring up the CAN-over-USB adapter for the Ranger UGV.
if [ -f "${REPO_ROOT}/src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash" ]; then
    sudo bash "${REPO_ROOT}/src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash"
else
    echo "[setup.sh] WARNING: ranger_ros2 not found. Run 'vcs import src < dependencies.repos' first."
fi

# KSJ 3D laser profiler runtime libraries.
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${REPO_ROOT}/src/laser_scanner/include/KSJApi.bin/x64"
