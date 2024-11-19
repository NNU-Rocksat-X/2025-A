#!/usr/bin/env bash

# Runs daedalus mission on after bootup
# Add the hostname of the device in the first argument
source ~/catkin_ws/devel/setup.bash

ROS_MASTER_URI=http://daedalus1:11311
ROS_HOSTNAME=$1
echo "Launching for $1"

ARM1_HOSTNAME="daedalus1"

roscore &
sleep 20
# roslaunch apogee_vision record.launch &
roslaunch arm2v4_description config.launch 
roslaunch apogee_robot_core arm2d2.launch ARM:=ARM1 > /dev/null &
roslaunch daedalus_core ARM1_mission.launch

# if [ "$ROS_HOSTNAME" == daedalus1 ]; then
#     roscore &
#     roslaunch arm2v4_description config.launch
#     roslaunch apogee_vision record.launch &
#     roslaunch apogee_robot_core arm2d2.launch ARM:=ARM1 > /dev/null &
#     roslaunch daedalus_core ARM1_mission.launch
# else
#     roslaunch arm2v4_description config.launch
#     roslaunch apogee_robot_core arm2d2.launch ARM:=ARM2 > /dev/null & 
#     roslaunch daedalus_core ARM2_mission.launch 
# fi

