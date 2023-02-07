#include <ros/ros.h>
#include <Eigen/Dense>

#include "apogee_vision/VelocityFilter.h"
//#include "apogee_vision/AngularFilter.h"
#include "apogee_vision/ObjectDynamics.h"
#include "apogee_vision/OrientationPredictor.h"

// MSG & SRVs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Trigger.h>
#include <daedalus_msgs/OrientationState.h>

// TODO: Replace with daedalus version
#include <marsha_msgs/PredictPosition.h> 
#include <marsha_msgs/ObjectObservation.h>


void point_to_v3f(const geometry_msgs::Point msg, Eigen::Vector3f &vector)
{
    vector = Eigen::Vector3f(msg.x, msg.y, msg.z);
}

void quat_to_v4f(const geometry_msgs::Quaternion msg, Eigen::Vector4f &vector)
{
    vector = Eigen::Vector4f(msg.x, msg.y, msg.z, msg.w);
}


// Each filter should have a filter interface object which collects external parameters 
// and provides an interface for the predictor.
// Then there is a predictor which contains the ros services
class Predictor {
    private:
        VelocityEstimator* pos_filter;

        Eigen::Vector3f position;
        Eigen::Vector4f orientation;
        Eigen::Vector3f orientation_prediction; // euler
        ros::Time init_time;

        ros::Time predict_time = ros::Time(0); // Time at which prediction starts

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
            point_to_v3f(msg->position, position);
            quat_to_v4f(msg->orientation, orientation);

            if (!position_recvd)
            {
                position_recvd = true;
                init_time = ros::Time::now();
            }

        }


        void publish_orientation_state(Eigen::Matrix<float, 7, 1> orientation_estimate, Eigen::Matrix<float, 6, 6> orientation_covariance)
        {
            daedalus_msgs::OrientationState msg;
            for (int i = 0; i < 7; i++)
            {
                msg.orientation.push_back(orientation_estimate(i));
            }

            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    msg.covariance.push_back(orientation_covariance(i,j));

                }
            }
            or_pub.publish(msg);
        }

        // Currently only works for position
        bool observe(marsha_msgs::ObjectObservation::Request &req,
                     marsha_msgs::ObjectObservation::Response &res)
        {
            Position::State state;
            pos_filter->get_state(state);

            geometry_msgs::Point pmsg;
            pmsg.x = state.position[0];
            pmsg.y = state.position[1];
            pmsg.z = state.position[2];

            geometry_msgs::Vector3 vmsg;
            vmsg.x = state.velocity[0];
            vmsg.y = state.velocity[1];
            vmsg.z = state.velocity[2];

            res.position = pmsg;
            res.velocity = vmsg;
            return true;
        }



        
        bool prediction_ready(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res)
        {
            res.success = has_converged;
            return true;
        }

        bool predict(marsha_msgs::PredictPosition::Request &req,
                     marsha_msgs::PredictPosition::Response &res)
        {
            float t = 1;
            Eigen::Quaternionf orientation = predict_orientation(t);
            orientation_prediction = quat_to_euler(orientation);

            predict_time = ros::Time::now() + ros::Duration(t);
            /*
            Position::State state;
            pos_filter->get_state(state);

            Searcher searcher(state.position, state.velocity, state.acceleration);

            // Find time until object is at closest point to base of arm
            float delta_time = searcher.solve();

            Position::State state_prediction;
            pos_filter->predict_state(delta_time, state_prediction);

            geometry_msgs::Point position_msg;
            position_msg.x = state_prediction.position(0);
            position_msg.y = state_prediction.position(1);
            position_msg.z = state_prediction.position(2);
            res.position = position_msg;
            res.predicted_time.data = ros::Time::now() + ros::Duration(delta_time);
            */

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

            pos_filter = new VelocityEstimator(state_guess, initial_uncertainty_guess, rate_period, acceleration_variance, measurement_variance);


            // -----------------------------------------------

            ros::Rate rate(rate_freq);

            Position::StateVector state_estimate;
            Eigen::Matrix<float, 7, 1> orientation_estimate;
            Eigen::Matrix<float, 6, 6> orientation_covariance; // only used for orientation kalman filter


            ros::Time prediction_time;
            float prediction_duration = 5;
            Eigen::Matrix<float, 3, 1> prediction;

            ros::Time prev_time = ros::Time::now();

            while(ros::ok())
            {
                // Most of this loop is setup for debugging
                if (!has_converged)
                {
                    if (position_recvd)
                    {
                        //ROS_INFO("Waiting for convergence! Error: %f", abs(state_estimate(2)));
                        //ROS_INFO("==============================================================");
                        // calculate dt
                        ros::Time current_time = ros::Time::now();
                        ros::Duration dt = current_time - prev_time;
                        Eigen::Vector3f ang_vel = calculateAngularVelocity(orientation, dt.toSec());
                        prev_time = current_time;

                        orientation_estimate[4] = ang_vel[0];
                        orientation_estimate[5] = ang_vel[1];
                        orientation_estimate[6] = ang_vel[2];

                        if (predict_time != ros::Time(0))
                        {
                            if (ros::Time::now() > predict_time)
                            {
                                //ROS_INFO("Actual:");
                                //print_eigen(orientation);

                                Eigen::Vector3f vector_orientation = quat_to_euler(orientation);

                                ROS_INFO(" actual");
                                print_eigen(vector_orientation);

                                Eigen::Vector3f difference = vector_orientation - orientation_prediction;

                                predict_time = ros::Time(0);
                            }
                        }


                        //orientation_estimator->step(orientation);
                        //orientation_estimator->get_state(orientation_estimate, orientation_covariance);

                        publish_orientation_state(orientation_estimate, orientation_covariance);
                        
                        /*
                        pos_filter->step(position);
                        pos_filter->get_state(state_estimate);
                        if (abs(state_estimate(2)) < 0.005)
                        {
                            float converge_duration = (ros::Time::now() - init_time).toSec();
                            ROS_INFO("Converge_time: %f", converge_duration);
                            has_converged = true;
                        }*/
                        
                    } else {
                        ROS_INFO("Waiting for position");
                    }
                }
                else {
                    pos_filter->step(position);
                    pos_filter->get_state(state_estimate);

                    if(abs(state_estimate(2)) > 1)
                    {
                        has_converged = false;
                        init_time = ros::Time::now();
                    }
                }

                ros::spinOnce();
                rate.sleep();
            }

            ros::spin();
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

/*





*/