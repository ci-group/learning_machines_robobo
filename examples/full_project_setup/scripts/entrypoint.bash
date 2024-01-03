#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
source /root/catkin_ws/setup.bash

rosrun learning_machines learning_robobo_controller.py "$@"
