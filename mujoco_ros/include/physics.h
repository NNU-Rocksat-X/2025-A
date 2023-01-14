#include <mujoco.h>
#include <ros/ros.h>

#include <mujoco_ros/ApplyForce.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>


#ifndef PHYSICS_H_
#define PHYSICS_H_

struct Vector3 {
  double x;
  double y;
  double z;
};

struct Quaternion {
  double w;
  double x;
  double y;
  double z;
};

struct Force {
  int id;
  double* x;
  double* y;
  double* z;
  double* tx;
  double* ty;
  double* tz;
  double end_time;
};

// Model forces contains a force for each body
struct ModelForces {
  Force* forces;
};

static void reset_force(Force* force);

void set_body(int body_id, double x, double y, double z);
bool apply_force_cb(mujoco_ros::ApplyForce::Request &req,
                    mujoco_ros::ApplyForce::Response &res);
void initialize_forces();
void set_forces();
void get_model_pose(geometry_msgs::Pose* pose, int model_id);
void get_model_twist(geometry_msgs::Twist* twist, int model_id);
void get_model_twist_accel(geometry_msgs::Twist* twist, int model_id);

#endif /* PHYSICS_H_ */