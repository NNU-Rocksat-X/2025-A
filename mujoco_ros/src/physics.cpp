
#include "physics.h"

// Note there is an issue where an apply duration < 0.1 does not register reset force.
///////////////////////////////////////////////
///             GLOBAL VARIABLES
///////////////////////////////////////////////
extern mjModel* m;
extern mjData* d;

///////////////////////////////////////////////
///             STATIC VARIABLES
///////////////////////////////////////////////

static Force* model_forces;

///////////////////////////////////////////////
///             STATIC FUNCTIONS
///////////////////////////////////////////////

static void reset_force(Force* force)
{
  *force->x = 0;
  *force->y = 0;
  *force->z = 0;
  *force->tx = 0;
  *force->ty = 0;
  *force->tz = 0;
  force->end_time = -1;
}

///////////////////////////////////////////////
///             GLOBAL FUNCTIONS
///////////////////////////////////////////////

void set_body(int body_id, double x, double y, double z)
{
  reset_force(&model_forces[body_id]);

  // extract addresses from joint specification
  int qposadr = m->jnt_qposadr[m->body_jntadr[body_id]];
  int qveladr = m->jnt_dofadr[m->body_jntadr[body_id]];
  
  // Set position
  Vector3* pos = (Vector3*)&d->qpos[qposadr];
  pos->x = x;
  pos->y = y;
  pos->z = z;

  // Set velocity to 0
  Vector3* vel = (Vector3*)&d->qvel[qveladr];
  vel->x = 0;
  vel->y = 0;
  vel->z = 0;

  Vector3* ang_vel = (Vector3*)&d->qvel[qveladr + 3];
  ang_vel->x = 0;
  ang_vel->y = 0;
  ang_vel->z = 0;
}

bool apply_force_cb(mujoco_ros::ApplyForce::Request &req,
                    mujoco_ros::ApplyForce::Response &res)
{
  if (req.body_id > m->nbody)
  {
    ROS_ERROR("Body ID does not exist.");
    return false;
  } else {
    ROS_INFO("Setting Force!");
    double* body_ptr = d->xfrc_applied + req.body_id*6;
    Force* forceptr = &model_forces[req.body_id];
    *forceptr->x = req.wrench.force.x;
    *forceptr->y = req.wrench.force.y;
    *forceptr->z = req.wrench.force.z;
    *forceptr->tx = req.wrench.torque.x;
    *forceptr->ty = req.wrench.torque.y;
    *forceptr->tz = req.wrench.torque.z;
    forceptr->end_time = d->time + req.apply_duration;
  }
  return true;
}


void initialize_forces()
{
    model_forces = (Force*)std::malloc(sizeof(Force) * m->nbody);
    for (int i = 0; i < m->nbody; i++)
    {
        Force* forceptr = &model_forces[i];
        double* body_ptr = d->xfrc_applied + i*6;
        forceptr->id = i;
        forceptr->x = body_ptr;
        forceptr->y = body_ptr + 1;
        forceptr->z = body_ptr + 2;
        forceptr->tx = body_ptr + 3;
        forceptr->ty = body_ptr + 4;
        forceptr->tz = body_ptr + 5;
        forceptr->end_time = 0;
    }
}


void set_forces()
{
  for (int i = 0; i < m->nbody; i++)
  {
    if (model_forces[i].end_time > d->time && model_forces[i].end_time != -1)
    {
      reset_force(&model_forces[i]);
      model_forces[i].end_time = -1;
      ROS_INFO("Resetting force.");
    }
  }
}

void get_model_pose(geometry_msgs::Pose* pose, int model_id)
{
  Vector3* position = (Vector3*)(d->xpos + model_id*3);
  Quaternion* orientation = (Quaternion*)(d->xquat + model_id*4);

  pose->position.x = position->x;
  pose->position.y = position->y;
  pose->position.z = position->z;
  pose->orientation.x = orientation->x;
  pose->orientation.y = orientation->y;
  pose->orientation.z = orientation->z;
  pose->orientation.w = orientation->w;
}

void get_model_twist(geometry_msgs::Twist* twist, int model_id)
{
  double vel[6];
  mj_objectVelocity(m, d, mjOBJ_BODY, model_id, vel, false);

  Vector3* lin = (Vector3*)&vel[3];
  Vector3* ang = (Vector3*)&vel[0];

  twist->linear.x = lin->x;
  twist->linear.y = lin->y;
  twist->linear.z = lin->z;
  twist->angular.x = ang->x;
  twist->angular.y = ang->y;
  twist->angular.z = ang->z;
}

void get_model_twist_accel(geometry_msgs::Twist* twist, int model_id)
{
  double acc[6];
  mj_objectAcceleration(m, d, mjOBJ_BODY, model_id, acc, true);

  Vector3* lin = (Vector3*)&acc[0];
  Vector3* ang = (Vector3*)&acc[3];

  twist->linear.x = lin->x;
  twist->linear.y = lin->y;
  twist->linear.z = lin->z;
  twist->angular.x = ang->x;
  twist->angular.y = ang->y;
  twist->angular.z = ang->z;
}