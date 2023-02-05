#ifndef _EIGEN_UTIL_H_
#define _EIGEN_UTIL_H_


#include <ros/ros.h>
#include <Eigen/Dense>

Eigen::Vector4f quat_to_v4(const Eigen::Quaternionf q);

void print_eigen(const Eigen::Ref<const Eigen::MatrixXf>& mat);
void print_eigen(const Eigen::Quaternionf q);

Eigen::Quaternionf v4_to_quat(Eigen::Vector4f v);

Eigen::Vector3f quat_to_v3(Eigen::Quaternionf q);

Eigen::Quaternionf v3_to_quat(Eigen::Vector3f v);

#endif /*_EIGEN_UTIL_H_*/