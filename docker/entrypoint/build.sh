#!/bin/bash

source /opt/ros/humble/setup.bash
source /root/ros2-ws/install/setup.bash

# Define a file to indicate if rosdep update has been run
ROSDEP_UPDATE_FLAG="/root/.rosdep_updated"

# update rosdep
if [ ! -f "$ROSDEP_UPDATE_FLAG" ]; then
  cd /root/ros2-ws
  apt update
  rosdep install --from-paths src --ignore-src -r -y 
  touch "$ROSDEP_UPDATE_FLAG"
else
  echo "rosdep update already run, skipping..."
fi

# build 
cd /root/ros2-ws
colcon build

tail -f /dev/null