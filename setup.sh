source /opt/ros/humble/setup.sh
source install/local_setup.sh

eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

export IP_address=192.168.201.1
export DOBOT_TYPE=cr20
 
