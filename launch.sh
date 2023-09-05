#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <webots_port>" >&2
  exit 1
fi

export WEBOTS_PORT=$1

source install/local_setup.sh

echo "start in ${WEBOTS_PORT}"
ros2 launch uav_launch robot_launch.py
