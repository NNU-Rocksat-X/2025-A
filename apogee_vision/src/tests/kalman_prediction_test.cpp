#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include "apogee_vision/VelocityFilter.h"


VelocityEstimator* estimator;
Eigen::Vector3f position;
bool position_recvd = false;

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

    double current_time = ros::Time::now().toSec();
    position_recvd = true;

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


    int rate_freq = 30;
    float rate_period = 1.0 / (float)rate_freq;
    ROS_INFO("rate period: %f", rate_period);

    estimator = new VelocityEstimator(state_guess, 1000, rate_period);

    ros::Rate rate(rate_freq);

    Vector9f state_estimate;

    int counter;
    ros::Time prediction_time;
    float prediction_duration = 5;
    Vector3f prediction;

    while(ros::ok())
    {
        if (counter > 300)
        {
            
            estimator->predict_state(prediction_duration, prediction);
            prediction_time = ros::Time::now();
            ROS_INFO("Set time: %f", prediction_time.toSec() + prediction_duration);
            ROS_INFO("Position: x: %f y: %f z: %f", position[0], position[1], position[2]);
            ROS_INFO("Prediction: x: %f y: %f z: %f", prediction[0], prediction[1], prediction[2]);
            counter = -1;
        }

        // Currently testing prediction
        if (counter < 0)
        {
            if (ros::Time::now().toSec() > prediction_time.toSec() + prediction_duration)
            {
                // Check prediction accuracy
                ROS_INFO("Prediction time: %f", ros::Time::now().toSec());
                ROS_INFO("Position: x: %f y: %f z: %f", position[0], position[1], position[2]);
                ROS_INFO("Difference: x: %f, y: %f, z: %f", position[0] - prediction[0], position[1] - prediction[1], position[2] - prediction[2]);
                counter = 0;
            }
        } else {
            if (position_recvd)
            {
                counter++;
                estimator->step(position);
                ROS_INFO("Counter: %i", counter);
                estimator->get_state(state_estimate);
                ROS_INFO("State estimate");
                estimator->print_vector9(state_estimate);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

}