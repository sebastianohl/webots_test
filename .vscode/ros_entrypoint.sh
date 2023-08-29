#!/bin/bash
set -e

export ROS_DISTRO=humble
export ROS_ROOT=/opt/ros/$ROS_DISTRO
export WEBOTS_HOME=/usr/local/webots
export SUMO_HOME=${WEBOTS_HOME}/projects/default/resources/sumo/

ros_env_setup="/opt/ros/$ROS_DISTRO/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"
echo "WEBOTS_HOME $WEBOTS_HOME"
echo "SUMO_HOME $SUMO_HOME"

/bin/bash
