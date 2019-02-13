#!/usr/bin/env bash

# Setup file for running a remote from the pnr_core computer.

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://$(echo $SSH_CLIENT | awk '{print $1}'):11311
export ROS_IP=192.168.1.5

exec "$@"
