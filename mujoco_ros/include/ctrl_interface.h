// ros_interface.h

#ifndef CTRL_INTERFACE_H_
#define CTRL_INTERFACE_H_

#include <mujoco.h>
#include <ros/ros.h>

#include <daedalus_msgs/TeensyMsg.h>
#include <daedalus_msgs/GraspDetect.h>
#include <mujoco_ros/SetBody.h>
#include <std_srvs/Trigger.h>

#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32.h>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Wrench.h>

#include <moveit_msgs/DisplayRobotState.h>

#include "physics.h"

#define MAX_JOINTS (7) // read this from joint_config.yaml (on rosparam)
#define PI         (3.14159)



void init_ros_node();

void ctrl_robot(const mjModel* m, mjData* d);

void init_robot_state(void);

void init_frame_pub(int w, int h);

void send_rgb(unsigned char* rgb);

void send_depth(float* depth);

void send_cam_pos(double* cam_pos, double* cam_quat);

void get_cam_pos(double* lookat, double dist, double azimuth, double elevation);

void send_object_state(void);

void ros_reset(void);


#endif /* CTRL_INTERFACE_H_ */

