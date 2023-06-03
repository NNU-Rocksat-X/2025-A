#ifndef _MOVEIT_INTERFACE_H_
#define _MOVEIT_INTERFACE_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Dense>

#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <daedalus_msgs/TeensyMsg.h>

#include <daedalus_msgs/MoveCmd.h>
#include <daedalus_msgs/PositionCmd.h>
#include <daedalus_msgs/GetPos.h>
#include <daedalus_msgs/PostureCmd.h>
#include <daedalus_msgs/GraspDetect.h>
#include <daedalus_msgs/PlanGrasp.h>
#include <daedalus_msgs/ExecuteGrasp.h>


// Testing
#include <iostream>
#include <iterator>
// ------

#include <vector>

// delete
struct GraspPlan {
    bool planned;
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp;
    moveit::planning_interface::MoveGroupInterface::Plan grasp;
};

static const std::string ARM_PLANNING_GROUP = "manipulator";



class MoveInterface {
    private:
        moveit::planning_interface::MoveGroupInterface* move_group;

        // Used for toggling collision objects, could maybe be put in a different file
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        
        // Services provide an interface for any node to give the interface a command
        ros::ServiceServer poseService;
        ros::ServiceServer asyncPoseService;
        ros::ServiceServer jointPoseService;
        ros::ServiceServer asyncJointPoseService;
        ros::ServiceServer positionService;
        ros::ServiceServer getPosService;
        ros::ServiceServer getJointService;
        ros::ServiceServer postureService;
        ros::ServiceServer graspService;
        ros::ServiceServer planGraspService;
        ros::ServiceServer executeGraspService;

        ros::Publisher graspPub;
        ros::Publisher display_publisher;
        ros::Publisher posePub; // added publisher for the joint position cmds


        std::string pose_param;
        std::string joint_param;
        
        int num_joints = 8;

        // Default RTT planning timeout
        float ik_timeout = 0.07;


        bool grasp(std::string pose);
        bool wait_until_complete(std::vector<double> joint_cmds);
        bool wait_until_grasp_complete(float grasp_position);

        GraspPlan grasp_plan;

    public:
        MoveInterface(ros::NodeHandle *nh);
        ~MoveInterface(void) {}

        bool graspCmd(daedalus_msgs::MoveCmd::Request &req,
                daedalus_msgs::MoveCmd::Response &res);

        bool getPose(daedalus_msgs::GetPos::Request &req,
                     daedalus_msgs::GetPos::Response &res);

        bool get_joints(std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response & res);

        bool poseCmd(daedalus_msgs::MoveCmd::Request &req,
                     daedalus_msgs::MoveCmd::Response &res);

        bool asyncPoseCmd(daedalus_msgs::MoveCmd::Request &req,
                          daedalus_msgs::MoveCmd::Response &res);

        bool jointPoseCmd(daedalus_msgs::MoveCmd::Request &req,
                          daedalus_msgs::MoveCmd::Response &res);

        bool asyncJointPoseCmd(daedalus_msgs::MoveCmd::Request &req,
                               daedalus_msgs::MoveCmd::Response &res);

        bool planGrasp(daedalus_msgs::PlanGrasp::Request &req,
                       daedalus_msgs::PlanGrasp::Response &res);

        bool executeGrasp(daedalus_msgs::ExecuteGrasp::Request &req,
                          daedalus_msgs::ExecuteGrasp::Response &res);

        bool postureCmd(daedalus_msgs::PostureCmd::Request &req,
                        daedalus_msgs::PostureCmd::Response &res);

        bool positionCmd(daedalus_msgs::PositionCmd::Request &req,
                         daedalus_msgs::PositionCmd::Response &res);

};


#endif /* _MOVEIT_INTERFACE_H_ */