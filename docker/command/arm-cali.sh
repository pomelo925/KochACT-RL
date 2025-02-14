#!/bin/bash

####################  ARGS  ###################
CONFIG_FILE_PATH="/root/ros2-ws/src/koch_ros2_wrapper/config/two_leader_follower.yaml"
###############################################

source /opt/ros/humble/setup.bash
source /root/ros2-ws/install/setup.bash

ros2 run koch_ros2_wrapper koch_calibration --ros-args -p config_file:=$CONFIG_FILE_PATH &
ros2 run topic_tools relay /left_leader/joint_states /left_follower/joint_states_control

exec "$@"