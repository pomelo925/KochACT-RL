#!/bin/bash

source /opt/ros/humble/setup.bash
source /root/ros2-ws/install/setup.bash

cd /root/ros2-ws
apt update
rosdep install --from-paths src --ignore-src -r -y 
colcon build

tail -f /dev/null