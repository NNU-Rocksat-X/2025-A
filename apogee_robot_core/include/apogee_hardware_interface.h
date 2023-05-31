#ifndef ARM2D2_HARDWARE_INTERFACE_H
#define ARM2D2_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <string.h>
#include <algorithm>
//#include <termios.h>

#include <std_msgs/Int16.h>
#include <daedalus_msgs/TeensyMsg.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <termios.h>
#include <fstream>
#include <iostream>

#define HEADSTART (0.2)

class Arm2D2Interface : public hardware_interface::RobotHW 
{
    public:
        Arm2D2Interface(ros::NodeHandle &nh_);
        ~Arm2D2Interface();

        void read();
        void write();
        void update_time(ros::Time time);

        void encoderCallBack(const moveit_msgs::DisplayRobotState &msg);
        void otherEncoderCallBack(const moveit_msgs::DisplayRobotState &msg);
        void trajectory_cb(const control_msgs::FollowJointTrajectoryActionGoal &msg);
        void update_setpoint(void);
        //void graspCallBack(const std_msgs::Bool &msg);
    private:
        ros::NodeHandle nh;
        ros::Publisher step_pub;
        ros::Subscriber enc_sub;
        ros::Subscriber other_enc_sub;
        ros::Subscriber trajectory_goal_sub;

        ros::Time previous_time;
        ros::Time current_time;

        
        //ros::Subscriber grip_sub;


        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::JointStateInterface other_joint_state_interface;
        hardware_interface::PositionJointInterface joint_control_interface;

        // Note: This isnt ideal because this 9 is static but num joints is dynamic
        double fake_cmd[15];
        double cmd[15];
        std::vector<std::map<std::string, double>> position_setpoints;
        std::vector<double> setpoint_times;
        int current_trajectory_point = 0;

        double pos[15];
        double vel[15];
        double eff[15];

        double other_pos[15];
        double other_vel[15];
        double other_eff[15];


        std::vector<std::map<std::string, float>> gains;

        int num_joints = 0;
        int num_grip_joints = 0;
        int other_num_joints = 0;

        std::vector<std::string> joint_names;
        std::vector<float> deg_per_steps;

        double pid_controller(int joint_id, double position_cmd);

        // These should be static and only in the cpp file
        double radToDeg(double rad);
        double degToRad(double deg);
};

#endif