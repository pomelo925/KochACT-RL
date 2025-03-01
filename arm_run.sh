#!/bin/bash

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
    cd docker/arm
    docker-compose -p koch-act up raw -d
    ;;
  ws-build)
    cd docker/arm
    docker-compose -p koch-act up ws-build -d
    ;;
  arm-cali)
    cd docker/arm
    docker-compose -p koch-act up arm-cali -d
    ;;
  arm-sync)
    cd docker/arm
    docker-compose -p koch-act up arm-sync -d
    ;;
  armv-sync)
    cd docker/arm
    docker-compose -p koch-act up armv-sync -d
    ;;
  armv-rec)
    cd docker/arm
    docker-compose -p koch-act up armv-rec -d
    ;;
  train)
    cd docker/arm
    docker-compose -p koch-act up train -d
    ;;
  deploy)
    cd docker/arm
    docker-compose -p koch-act up deploy -d
    ;;
  *)
    echo "Invalid argument: $1"
    echo "Usage: $0 {raw|ws-build|arm-cali|arm-sync|armv-sync|armv-rec|train|deploy}"
    exit 1
    ;;
esac