#!/bin/bash

source install/local_setup.sh

echo "start in ${WEBOTS_PORT}"
ros2 launch uav_launch robot_launch.py
