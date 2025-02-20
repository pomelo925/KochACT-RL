#!/bin/bash

#######################################################################################
### This script provides 8 modes to interact with the KochACT-RL environment:
### 
### Modes:
###   raw:        Start the environment without running any nodes.
###   ws-build:   Build ROS2 workspace.
###   arm-cali:   Calibrate Koch robot arms.
###   arm-sync:   Synchronize Koch robot arms.
###   armv-sync:  Synchronize arms & turn on visual sensors.
###   armv-rec:   Record Koch robot arms and visual data.
###   train:      Train your custom dataset with the selected model.
###   deploy:     Deploy your custom model on real robot.
###
### Usage:
###   ./gpu_run.sh {raw|ws-build|arm-cali|arm-sync|armv-sync|armv-rec|train|deploy}
#######################################################################################

# Add docker to xhost
xhost +local:docker

# Set display
export DISPLAY=:0

# Make all entrypoint files executable
chmod +x docker/entrypoint/*

# Check validity of args
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 {raw|ws-build|arm-cali|arm-sync|armv-sync|armv-rec|train|deploy}"
  exit 1
fi

# Run docker compose
case "$1" in
  raw)
    cd docker/gpu
    docker compose -p koch-actpp up raw -d
    ;;
  ws-build)
    cd docker/gpu
    docker compose -p koch-actpp up ws-build -d
    ;;
  arm-cali)
    cd docker/gpu
    docker compose -p koch-actpp up arm-cali -d
    ;;
  arm-sync)
    cd docker/gpu
    docker compose -p koch-actpp up arm-sync -d
    ;;
  armv-sync)
    cd docker/gpu
    docker compose -p koch-actpp up armv-sync -d
    ;;
  armv-rec)
    cd docker/gpu
    docker compose -p koch-actpp up armv-rec -d
    ;;
  train)
    cd docker/gpu
    docker compose -p koch-actpp up train -d
    ;;
  deploy)
    cd docker/gpu
    docker compose -p koch-actpp up deploy -d
    ;;
  *)
    echo "Invalid argument: $1"
    echo "Usage: $0 {raw|ws-build|arm-cali|arm-sync|armv-sync|armv-rec|train|deploy}"
    exit 1
    ;;
esac