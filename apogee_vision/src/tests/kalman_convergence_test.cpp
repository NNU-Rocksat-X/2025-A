#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include "apogee_vision/VelocityFilter.h"


VelocityEstimator* estimator;
Eigen::Vector3f position;
bool position_recvd = false;
ros::Time init_time;

void point_to_v3f(const geometry_msgs::Point msg, Eigen::Vector3f &vector)
{
    vector = Eigen::Vector3f(msg.x, msg.y, msg.z);
}

void quat_to_v4f(const geometry_msgs::Quaternion msg, Eigen::Vector4f &vector)
{
    vector = Eigen::Vector4f(msg.x, msg.y, msg.z, msg.w);
}

void pose_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
    Eigen::Vector4f orientation;
    point_to_v3f(msg->position, position);
    quat_to_v4f(msg->orientation, orientation);

    if (!position_recvd)
    {
        position_recvd = true;
        init_time = ros::Time::now();
    }

}

int main(int argc, char** argv)
{
    // Ros initialization
    ros::init(argc, argv, "trajectory_predictor");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/object_pose", 1, pose_cb);

    Vector9f state_guess = Vector9f::Zero();

    // Kinda close guess
    state_guess(0) = -0.2;
    state_guess(3) = -0.1;
    state_guess(6) = 0.05;

    // Ros parameters
    int rate_freq;
    float acceleration_variance;
    float measurement_variance;
    float initial_uncertainty_guess;

    if (nh.hasParam("/rates/trajectory_predictor")) 
    {
        nh.getParam("/rates/trajectory_predictor", rate_freq);
    } else {
        ROS_ERROR("Rate Parameters not loaded.");
    }
    if (nh.hasParam("TrajectoryPredictorConfig/acceleration_variance"))
    {
        nh.getParam("TrajectoryPredictorConfig/acceleration_variance", acceleration_variance);
        nh.getParam("TrajectoryPredictorConfig/measurement_variance", measurement_variance);
        nh.getParam("TrajectoryPredictorConfig/initial_uncertainty_guess", initial_uncertainty_guess);
    } else {
        ROS_ERROR("TrajectoryPredictorConfig parameters not loaded.");
    }
    
    float rate_period = 1.0 / (float)rate_freq;
    ROS_INFO("rate period: %f", rate_period);

    estimator = new VelocityEstimator(state_guess, initial_uncertainty_guess, rate_period, acceleration_variance, measurement_variance);

    ros::Rate rate(rate_freq);

    Vector9f state_estimate;

    int counter;
    ros::Time prediction_time;
    float prediction_duration = 5;
    Vector3f prediction;

    while(ros::ok())
    {
        if (counter > -1)
        {
            if (position_recvd)
            {
                counter++;
                estimator->step(position);
                //ROS_INFO("Counter: %i", counter);
                estimator->get_state(state_estimate);
                //ROS_INFO("State estimate");
                //estimator->print_vector9(state_estimate);
                if (abs(state_estimate(2)) < 0.00001)
                {
                    float converge_duration = (ros::Time::now() - init_time).toSec();
                    ROS_INFO("Converge_time: %f", converge_duration);
                    counter = -2;
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

}