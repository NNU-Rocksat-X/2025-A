#include <ros/ros.h>
#include <Eigen/Dense>

#include "apogee_vision/PositionFilter.h"
#include "apogee_vision/OrientationPredictor.h"
#include "apogee_vision/TrajectorySearch.h"

// MSG & SRVs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Trigger.h>
#include <daedalus_msgs/OrientationState.h>


#include <daedalus_msgs/PredictPosition.h> 
#include <daedalus_msgs/ObjectObservation.h>


Eigen::Vector3f point_to_v3f(const geometry_msgs::Point msg)
{
    Eigen::Vector3f vector(msg.x, msg.y, msg.z);
    return vector;
}

Eigen::Vector4f quat_to_v4f(const geometry_msgs::Quaternion msg)
{
    Eigen::Vector4f vector(msg.x, msg.y, msg.z, msg.w);
    return vector;
}

geometry_msgs::Quaternion quat_to_msg(const Eigen::Quaternionf q)
{
    geometry_msgs::Quaternion msg;
    msg.w = q.w();
    Eigen::Vector3f v = q.vec();
    msg.x = v[0];
    msg.y = v[1];
    msg.z = v[2];
    return msg;
}


// Each filter should have a filter interface object which collects external parameters 
// and provides an interface for the predictor.
// Then there is a predictor which contains the ros services
class Predictor {
    private:
    PositionFilter* pos_filter;
    OrientationPredictor* ori_predictor;

    Eigen::Vector3f measured_position;
    Eigen::Vector4f measured_orientation;
    ros::Time init_time;

    bool position_recvd = false;
    bool has_converged = false;

    // Ros comm
    ros::NodeHandle* nh;
    ros::Subscriber sub;
    ros::Publisher or_pub;
    ros::ServiceServer convergenceService;
    ros::ServiceServer predictService;
    ros::ServiceServer observeService;


    public:
    Predictor(ros::NodeHandle* nh_)
    {
        nh = nh_;

        sub = nh->subscribe("mujoco/object_pose", 1, &Predictor::pose_cb, this);
        or_pub = nh->advertise<daedalus_msgs::OrientationState>("/orientation_state", 10);
        convergenceService = nh->advertiseService("trajectory_predictor/ready", &Predictor::prediction_ready, this);
        predictService = nh->advertiseService("trajectory_predictor/predict", &Predictor::predict, this);
        observeService = nh->advertiseService("trajectory_predictor/observe", &Predictor::observe, this);

    }

    void pose_cb(const geometry_msgs::Pose::ConstPtr& msg)
    {
        measured_position = point_to_v3f(msg->position);
        measured_orientation = quat_to_v4f(msg->orientation);

        if (!position_recvd)
        {
            position_recvd = true;
            init_time = ros::Time::now();
        }

    }

    // Currently only works for position
    bool observe(daedalus_msgs::ObjectObservation::Request &req,
                    daedalus_msgs::ObjectObservation::Response &res)
    {
        Position::State state;
        pos_filter->get_state(state);

        Eigen::Quaternionf orientation;
        Eigen::Vector3f angular_velocity;
        ori_predictor->get_state(orientation, angular_velocity);

        geometry_msgs::Pose pmsg;
        // Position
        pmsg.position.x = state.position[0];
        pmsg.position.y = state.position[1];
        pmsg.position.z = state.position[2];

        // Orientation
        pmsg.orientation.w = orientation.w();
        pmsg.orientation.x = orientation.vec()[0];
        pmsg.orientation.y = orientation.vec()[1];
        pmsg.orientation.z = orientation.vec()[2];

        geometry_msgs::Twist vmsg;
        // Velocity
        vmsg.linear.x = state.velocity[0];
        vmsg.linear.y = state.velocity[1];
        vmsg.linear.z = state.velocity[2];

        // Angular velocity
        vmsg.angular.x = angular_velocity[0];
        vmsg.angular.y = angular_velocity[1];
        vmsg.angular.z = angular_velocity[2];

        res.pose = pmsg;
        res.velocity = vmsg;
        return true;
    }



    
    bool prediction_ready(std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response &res)
    {
        res.success = has_converged;
        return true;
    }

    bool predict(daedalus_msgs::PredictPosition::Request &req,
                    daedalus_msgs::PredictPosition::Response &res)
    {
        // Get current state
        Position::State curr_state;
        pos_filter->get_state(curr_state);

        // Setup to convert trajectory slice to position and delta time
        Searcher searcher(curr_state.position, curr_state.velocity, curr_state.acceleration);

        // Convert trajectory slice
        float delta_time;
        Vector3f predicted_position = searcher.solve(req.trajectory_slice, delta_time);

        // Orientation prediction
        Eigen::Quaternionf orientation = ori_predictor->predict(delta_time);

        // Fill in response msg
        geometry_msgs::Point position_msg;
        position_msg.x = predicted_position(0);
        position_msg.y = predicted_position(1);
        position_msg.z = predicted_position(2);
        res.position = position_msg;  
        res.orientation = quat_to_msg(orientation);
        res.delta_time = delta_time;

        return true;
    }
    
    void run()
    {
        Position::StateVector state_guess = Position::StateVector::Zero();

        // Kinda close guess
        state_guess(0) = -0.2;
        state_guess(3) = -0.1;
        state_guess(6) = 0.05;

        // -------------------- Velocity Estimator -----------------------
        int rate_freq;
        float acceleration_variance;
        float measurement_variance;
        float initial_uncertainty_guess;

        if (nh->hasParam("/rates/trajectory_predictor")) 
        {
            nh->getParam("/rates/trajectory_predictor", rate_freq);
        } else {
            ROS_ERROR("Rate Parameters not loaded.");
        }
        if (nh->hasParam("TrajectoryPredictorConfig/acceleration_variance"))
        {
            nh->getParam("TrajectoryPredictorConfig/acceleration_variance", acceleration_variance);
            nh->getParam("TrajectoryPredictorConfig/measurement_variance", measurement_variance);
            nh->getParam("TrajectoryPredictorConfig/initial_uncertainty_guess", initial_uncertainty_guess);
        } else {
            ROS_ERROR("TrajectoryPredictorConfig parameters not loaded.");
        }
        
        float rate_period = 1.0 / (float)rate_freq;
        ROS_INFO("rate period: %f", rate_period);

        pos_filter = new PositionFilter(state_guess, initial_uncertainty_guess, rate_period, acceleration_variance, measurement_variance);
        ori_predictor = new OrientationPredictor(rate_period);

        // ----------------------------------------------------------------

        ros::Rate rate(rate_freq);

        

        ros::Time prediction_time;
        float prediction_duration = 5;
        Eigen::Matrix<float, 3, 1> prediction;

        while(ros::ok())
        {

            if (!has_converged)
            {
                if (position_recvd)
                {
                    ori_predictor->step(measured_orientation);
            
                    // Step prediction
                    Position::StateVector state_estimate;
                    pos_filter->step(measured_position);
                    pos_filter->get_state(state_estimate);

                    // Test for convergence
                    if (abs(state_estimate(2)) < 0.005)
                    {
                        float converge_duration = (ros::Time::now() - init_time).toSec();
                        ROS_INFO("Converge_time: %f", converge_duration);
                        has_converged = true;
                    }
                    
                } else {
                    ROS_INFO("Waiting for position");
                }
            }
            else {
                
                ori_predictor->step(measured_orientation);

                // Step position
                Position::StateVector state_estimate;

                pos_filter->step(measured_position);
                pos_filter->get_state(state_estimate);

                // Test for convergence
                if(abs(state_estimate(2)) > 1)
                {
                    has_converged = false;
                    init_time = ros::Time::now();
                }
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

};




int main(int argc, char** argv)
{
    // Ros initialization
    ros::init(argc, argv, "trajectory_predictor");
    ros::NodeHandle nh;

    Predictor predictor = Predictor(&nh);
    predictor.run();

}

