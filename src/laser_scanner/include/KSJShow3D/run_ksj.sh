#!/usr/bin/env bash
# Auto-generated launch script for KSJShow3D
cd "$(dirname "${BASH_SOURCE[0]}")"
sudo env LD_LIBRARY_PATH=/home/cats/Dobot_Robot_Arm/Development-Linux_x64/KSJShow3D/../KSJApi.bin/x64:./ ./KSJShow3D "$@"
