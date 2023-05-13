#!/usr/bin/env bash

# Runs daedalus mission on after bootup
# Add the hostname of the device in the first argument
source ~/catkin_ws/devel/setup.bash

ROS_MASTER_URI=http://daedalus1:11311
ROS_HOSTNAME=$1
echo "Launching..."

if [[$1 == "daedalus1"]]; 
then
    roslaunch daedalus_core ARM1_mission.launch
else
    roslaunch daedalus_core ARM2_mission.launch
fi
