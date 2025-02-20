#!/bin/bash

CONFIG_FILE_PATH="/root/ros2-ws/src/koch_ros2_wrapper/config/two_leader_follower.yaml"

source /opt/ros/humble/setup.bash
source /root/ros2-ws/install/setup.bash

# sync koch robot arm
ros2 run koch_ros2_wrapper koch_leader_follower --ros-args -p config_file:=$CONFIG_FILE_PATH &
ros2 run topic_tools relay /left_leader/joint_states /left_follower/joint_states_control &
ros2 run topic_tools relay /right_leader/joint_states /right_follower/joint_states_control &

# turn on realsense camera
ros2 launch koch_ros2_wrapper multi_d415.launch.py