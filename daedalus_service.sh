#!/usr/bin/env bash

# Runs daedalus mission on after bootup
source ~/catkin_ws/devel/setup.bash

ROS_MASTER_URI=http://daedalus1:11311
ROS_HOSTNAME=daedalus1
echo "Launching..."
roslaunch daedalus_core ARM1_mission.launch
