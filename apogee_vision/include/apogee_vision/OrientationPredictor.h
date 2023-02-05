#ifndef _ORIENTATION_PREDICTOR_H_
#define _ORIENTATION_PREDICTOR_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "apogee_vision/EigenUtil.h"


Eigen::Quaternionf predict_orientation(float dt, float t);
Eigen::Vector3f calculateAngularVelocity(Eigen::Vector4f q_vec, float dt);


#endif /* _ORIENTATION_PREDICTOR_H_ */