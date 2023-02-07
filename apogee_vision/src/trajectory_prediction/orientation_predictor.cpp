
#include "apogee_vision/OrientationPredictor.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::CauchyLoss;

#define NUM_AXIS (3)

Eigen::Quaternionf prev_q;
Eigen::Quaternionf curr_q;
std::vector<Eigen::Vector3f> w_list;
std::vector<double> times;
double curr_time = 0;


///////////////////////////////////////////////////////////////////////////////////////
//                          CALCULATE ORIENTATION VELOCITY
///////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f calculateAngularVelocity(Eigen::Vector4f q_vec, double dt)
{
    static Eigen::Vector3f prev_vel;

    // q = current orientation
    curr_q = v4_to_quat(q_vec);
    
    // Change in orientation
    Eigen::Quaternionf q_diff = curr_q * prev_q.inverse();

    Eigen::Vector3f vector_diff = q_diff.toRotationMatrix().eulerAngles(0,1,2);
    Eigen::Vector3f angular_velocity = 2 *vector_diff / dt;


    prev_q = curr_q;
    curr_time += dt;
    w_list.push_back(angular_velocity);
    times.push_back(curr_time);

    // Only keep the most recent otherwise it takes to long to predict
    if (w_list.size() > 75)
    {
        w_list.erase(w_list.begin());
        times.erase(times.begin());
    }


    return angular_velocity;
}


///////////////////////////////////////////////////////////////////////////////////////
//                         PREDICT ORIENTATION
///////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3f calculate_avg_w(void)
{
    Eigen::Vector3f vel_sum(0, 0, 0);
    for (int i = 0; i < w_list.size(); i++)
    {
        for (int axis = 0; axis < NUM_AXIS; axis++)
        {
            vel_sum[axis] = vel_sum[axis] + w_list[i][axis];
        }
    }

    Eigen::Vector3f avg_vel = vel_sum / w_list.size();
    return avg_vel;
}

Eigen::Quaternionf predict_orientation(float t)
{
    ROS_INFO("----------");
    Eigen::Vector3f w = calculate_avg_w();
    Eigen::Vector3f last_w = w_list.back();
    ROS_INFO("avg vel");
    print_eigen(w);

    ROS_INFO("last vel");
    print_eigen(last_w);

    Eigen::Vector3f scaled_w = w * t;

    Eigen::Quaternionf q_dot(0, w[0], w[1], w[2]);

    Eigen::Quaternionf predicted_q = curr_q * q_dot;
    predicted_q.normalize();


    ROS_INFO("predicted q");
    print_eigen(quat_to_euler(predicted_q));

    
}