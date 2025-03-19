#!/bin/bash

# Add docker to xhost
xhost +local:docker

# Set display
export DISPLAY=:0

# Make all entrypoint files executable
sudo chmod +x docker/entrypoint/*

# Check validity of args
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 {raw|build|cali|teleop|cam|sync|train|deploy}"
  echo "raw\t- Launch container without running any service"
  echo "build\t- Build ros2 workspace"
  echo "cali\t- Calibrate Koch Robot Arm"
  echo "teleop\t- Teleoperate Koch Robot Arm"
  echo "cam\t- Launch all camera nodes"
  echo "sync\t- teleop + cam + synchorize related topics"
  echo "train\t- Train with custom dataset"
  echo "deploy\t- Deploy trained model"
  exit 1
fi

# Run docker compose
cd docker
case "$1" in
  raw|build|cali|teleop|cam|sync|train|deploy)
    docker compose -p KochACT-RL up "$1" -d
    ;;
  *)
    echo "Invalid argument: $1"
    echo "Usage: $0 {raw|build|cali|teleop|cam|sync|train|deploy}"
    exit 1
    ;;
esac