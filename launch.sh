#!/bin/sh

export WEBOTS_PORT=`./find_port.py`
xdg-open http://localhost?URL=ws://localhost:${WEBOTS_PORT}

export DISPLAY=:1
source install/local_setup.sh

echo "start in ${WEBOTS_PORT}"
ros2 launch uav_launch robot_launch.py
