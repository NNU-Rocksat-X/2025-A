#ifndef ARM2D2_HARDWARE_INTERFACE_H
#define ARM2D2_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string.h>

#include <std_msgs/Int16.h>
#include <daedalus_msgs/TeensyMsg.h>
#include <moveit_msgs/DisplayRobotState.h>


class Arm2D2Interface : public hardware_interface::RobotHW 
{
    public:
        Arm2D2Interface(ros::NodeHandle &nh_);
        ~Arm2D2Interface();

        void read();
        void write();
        void update_time(ros::Time time);

        void encoderCallBack(const moveit_msgs::DisplayRobotState &msg);
        //void graspCallBack(const std_msgs::Bool &msg);
    private:
        ros::NodeHandle nh;
        ros::Publisher step_pub;
        ros::Subscriber enc_sub;

        ros::Time previous_time;
        ros::Time current_time;

        
        //ros::Subscriber grip_sub;


        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::VelocityJointInterface joint_velocity_interface;

        // Note: This isnt ideal because this 9 is static but num joints is dynamic
        double cmd[15];

        double pos[15];
        double vel[15];
        double eff[15];

        
        int num_joints = 0;
        int num_grip_joints = 0;

        std::vector<std::string> joint_names;
        std::vector<float> deg_per_steps;


        // These should be static and only in the cpp file
        double radToDeg(double rad);
        double degToRad(double deg);
};

#endif