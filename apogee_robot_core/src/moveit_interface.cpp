/*
* Creates services so other components in the system can move the arm to pre-configured poses

* These services take a string as input which corresponds to a pose param file

* The service then performs the pose

* Author: Aaron Borger <borger.aaron@gmail.com>

*/

#include "moveit_interface.h"


MoveInterface::MoveInterface(ros::NodeHandle *nh) {
    move_group = new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP);

    ros::param::get(ros::this_node::getNamespace() + "/IK_timeout", ik_timeout);
    ROS_INFO("Planning time: %f", ik_timeout);
    move_group->setPlanningTime(ik_timeout);
    
    poseService = nh->advertiseService("pose_cmd", &MoveInterface::poseCmd, this);
    asyncPoseService = nh->advertiseService("async_pose_cmd", &MoveInterface::asyncPoseCmd, this);

    jointPoseService = nh->advertiseService("joint_pose_cmd", &MoveInterface::jointPoseCmd, this);
    asyncJointPoseService = nh->advertiseService("async_joint_pose_cmd", &MoveInterface::asyncJointPoseCmd, this);

    positionService = nh->advertiseService("position_cmd", &MoveInterface::positionCmd, this);

    getPosService = nh->advertiseService("get_pos", &MoveInterface::getPose, this);

    getJointService = nh->advertiseService("get_joints", &MoveInterface::get_joints, this);

    postureService = nh->advertiseService("posture_cmd", &MoveInterface::postureCmd, this);

    graspService = nh->advertiseService("grasp_cmd", &MoveInterface::graspCmd, this);

    planGraspService = nh->advertiseService("plan_grasp", &MoveInterface::planGrasp, this);

    executeGraspService = nh->advertiseService("execute_grasp", &MoveInterface::executeGrasp, this);

    graspPub = nh->advertise<std_msgs::Float32>("grip_position_cmd", 1);
    display_publisher = nh->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    ros::param::get(ros::this_node::getNamespace() + "/stepper_config/num_joints", num_joints);
    ROS_INFO("Running with %i joints", num_joints);

    pose_param = ros::this_node::getNamespace() + "/pose/";
    joint_param = ros::this_node::getNamespace() + "/joints/";

    grasp_plan.planned = false;
}

//////////////////////////////////////////////////////////////////
///                         HELPER FUNCTIONS                  
/////////////////////////////////////////////////////////////////
// Should probably move these to shared eigen_util

void print_eigen(const Eigen::Ref<const Eigen::MatrixXf>& mat)
{
    for (int row = 0; row < mat.rows(); row++)
    {
        std::string row_str = "";
        for (int col = 0; col < mat.cols(); col++)
        {
            row_str += std::to_string(mat(row, col));
            row_str += " ";
        }
        ROS_INFO("%s", row_str.c_str());
    }
}
Eigen::Vector4f quat_to_v4(const Eigen::Quaternionf q)
{
    Eigen::Vector4f v;
    v(0) = q.vec()(0);
    v(1) = q.vec()(1);
    v(2) = q.vec()(2);
    v(3) = q.w();
    return v;
}
void print_eigen(const Eigen::Quaternionf q)
{
    Eigen::Matrix<float, 4, 1> vector_q;
    vector_q = quat_to_v4(q);
    print_eigen(vector_q);
}

Eigen::Vector4f quat_to_v4f(const geometry_msgs::Quaternion msg)
{
    Eigen::Vector4f vector(msg.x, msg.y, msg.z, msg.w);
    return vector;
}

Eigen::Quaternionf msg_to_quat(const geometry_msgs::Quaternion msg)
{
    Eigen::Quaternionf q(msg.w, msg.x, msg.y, msg.z);
    return q;
}

Eigen::Vector3f rotate_vector(const Eigen::Vector3f v, const Eigen::Quaternionf q)
{
    //ROS_INFO("q:");
    //print_eigen(q);
    Eigen::Quaternionf quat_v = Eigen::Quaternionf(0, v[0], v[1], v[2]);
    Eigen::Quaternionf rotated_v = q*quat_v*q.conjugate();
    return rotated_v.vec();
}


geometry_msgs::Pose calculate_pre_grasp(const geometry_msgs::Pose grasp, float dist)
{
    Eigen::Vector3f up_vector(0, 0, 1);
    Eigen::Vector3f grasp_direction = rotate_vector(up_vector, msg_to_quat(grasp.orientation));
    grasp_direction.normalize();
    grasp_direction *= dist;
    //ROS_INFO("grasp_direction: ");
    //print_eigen(grasp_direction);

    geometry_msgs::Pose pre_grasp;
    pre_grasp.position.x = grasp.position.x - grasp_direction[0];
    pre_grasp.position.y = grasp.position.y - grasp_direction[1];
    pre_grasp.position.z = grasp.position.z - grasp_direction[2];
    pre_grasp.orientation = grasp.orientation;
    return pre_grasp;

}

//////////////////////////////////////////////////////////////////
///                         SERVICE CALLBACKS                  
/////////////////////////////////////////////////////////////////

/// @brief - Pose cmd takes name as request and moves to pre-programmed end effector pose
bool MoveInterface::poseCmd(daedalus_msgs::MoveCmd::Request &req,
                            daedalus_msgs::MoveCmd::Response &res)
{ 

    std::string param = pose_param + req.pose_name + "/";
    geometry_msgs::Pose target_pose;
    ROS_INFO("Going to pose: %s", param.c_str());


    ros::param::get(param + "position/x", target_pose.position.x);
    ros::param::get(param + "position/y", target_pose.position.y);       
    ros::param::get(param + "position/z", target_pose.position.z);
    ros::param::get(param + "orientation/x", target_pose.orientation.x);
    ros::param::get(param + "orientation/y", target_pose.orientation.y);
    ros::param::get(param + "orientation/z", target_pose.orientation.z);
    ros::param::get(param + "orientation/w", target_pose.orientation.w);

    ROS_INFO("Target_pose x: %f", target_pose.position.x);

    move_group->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan target_plan;

    bool success = (move_group->plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_DEBUG("Plan status: %s", success ? "SUCCESSFUL" : "FAILED");
    move_group->execute(target_plan);

    res.done = success;
    return true;            
}

/// @brief - Exact same as poseCmd, but doesn't block
bool MoveInterface::asyncPoseCmd(daedalus_msgs::MoveCmd::Request &req,
                                 daedalus_msgs::MoveCmd::Response &res)
{ 
    //std::string pose_name = req.pose_name;
    ROS_DEBUG("Going to pose: %s", req.pose_name.c_str());

    std::string param = pose_param + req.pose_name + "/";
    geometry_msgs::Pose target_pose;


    ros::param::get(param + "position/x", target_pose.position.x);
    ros::param::get(param + "position/y", target_pose.position.y);       
    ros::param::get(param + "position/z", target_pose.position.z);
    ros::param::get(param + "orientation/x", target_pose.orientation.x);
    ros::param::get(param + "orientation/y", target_pose.orientation.y);
    ros::param::get(param + "orientation/z", target_pose.orientation.z);
    ros::param::get(param + "orientation/w", target_pose.orientation.w);

    move_group->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan target_plan;

    bool success = (move_group->plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_DEBUG("Plan status: %s", success ? "SUCCESSFUL" : "FAILED");
    move_group->asyncExecute(target_plan);

    res.done = success;
    return true;            
}

bool MoveInterface::wait_until_complete(std::vector<double> joint_cmds)
{
    float POSITION_ACCURACY = 0.1;
    int MAX_WAIT = 100;
    std::vector<std::string> joint_names;
    ros::param::get("stepper_config/joint_names", joint_names);
    ros::param::get("stepper_config/goal_position_accuracy", POSITION_ACCURACY);
    ros::param::get("stepper_config/max_goal_time", MAX_WAIT);

    int joints_complete = 0;
    int cnt = 0;

    while (joints_complete < joint_cmds.size())
    {
        joints_complete = 0;
        robot_state::RobotState current_state(*move_group->getCurrentState());
        for (int i = 0; i < joint_cmds.size(); i++)
        {
            const double* joint_pos = current_state.getJointPositions(joint_names[i]);
            if (abs(*joint_pos - joint_cmds[i]) < POSITION_ACCURACY)
                joints_complete += 1;
        }

        if (cnt > MAX_WAIT*10)
        {
            return false;
        }
        cnt++;
        ros::Duration(0.1).sleep();
    }
    return true;
}


/// @brief - Takes name as request an moves to pre-programmed radian angles for each joint
///          Joint values in yaml file must be in radians
bool MoveInterface::jointPoseCmd(daedalus_msgs::MoveCmd::Request &req,
                                 daedalus_msgs::MoveCmd::Response &res)
{
    std::string param =  joint_param + req.pose_name;
    ROS_INFO("Going to joint state: %s", param.c_str());

    std::vector<double> joint_group_positions;

    if (ros::param::get(param, joint_group_positions))
    {
        ROS_INFO("Joint angles: %f, %f, %f", joint_group_positions[0], joint_group_positions[1], joint_group_positions[2]);


        bool target_success = move_group->setJointValueTarget(joint_group_positions);
        ROS_WARN("Target status: %s", target_success ? "SUCCESSFUL" : "FAILED");
        moveit::planning_interface::MoveGroupInterface::Plan target_plan;

        bool plan_success = (move_group->plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_DEBUG("Plan status: %s", plan_success ? "SUCCESSFUL" : "FAILED");

        if (plan_success) {
            move_group->execute(target_plan);
            bool completion_status = wait_until_complete(joint_group_positions);
            res.done = completion_status;
            return true;
        }
        else {
            res.done = false;
            return true;
        }
    } else {
        res.done = false;
        ROS_WARN("Joint pose does not exist!");
        return true;
    }


}

/// @brief - Exact same as jointPoseCmd, but doesn't block
bool MoveInterface::asyncJointPoseCmd(daedalus_msgs::MoveCmd::Request &req,
                    daedalus_msgs::MoveCmd::Response &res)
{ 
    std::string param =  joint_param + req.pose_name;
    ROS_INFO("Going to joint state: %s", param.c_str());

    std::vector<double> joint_group_positions;

    ros::param::get(param, joint_group_positions);


    bool target_success = move_group->setJointValueTarget(joint_group_positions);
    ROS_WARN("Target status: %s", target_success ? "SUCCESSFUL" : "FAILED");
    moveit::planning_interface::MoveGroupInterface::Plan target_plan;

    bool plan_success = (move_group->plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_DEBUG("Plan status: %s", plan_success ? "SUCCESSFUL" : "FAILED");


    if (plan_success) {
        move_group->asyncExecute(target_plan);
        res.done = true;
        return true;
    }
    else {
        res.done = false;
        return false;
    }            
}

// Moveit RobotModel also provides a distance function between different states which could be usefull for ensuring the planned path
// is optimized to the shortest path
// Look into planning grasp after pre-grasp is planned, but not executed
bool MoveInterface::planGrasp(daedalus_msgs::PlanGrasp::Request &req,
                              daedalus_msgs::PlanGrasp::Response &res) 
{
    // Open gripper before planning TODO: Ensure this does not block as that would slow down planning
    bool g_success = grasp("open");

    // Reset grasp plan
    grasp_plan.planned = false;

    const robot_state::JointModelGroup* joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
    geometry_msgs::Pose pre_grasp = calculate_pre_grasp(req.Grasp, 0.2);

    // Set start position
    robot_state::RobotState start_state(*move_group->getCurrentState());
    //start_state.setFromIK(joint_model_group, req.Grasp);
    move_group->setStartState(start_state);

    // Plan maneuver to preGrasp position
    move_group->setPoseTarget(pre_grasp);
    
    res.pre_grasp_success = (move_group->plan(grasp_plan.pre_grasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS); // Plan and check if succeeded
    ROS_INFO("Pre Grasp plan: %s", res.pre_grasp_success ? "SUCCESSFUL" : "FAILED");


    // TODO: Plan preGrasp and grasp simultaneously
    // Current method plans pregrasp then grasp if successfull, this causes the arm to move to preGrasp even if postGrasp isnt possible
    if (res.pre_grasp_success) {
        ROS_INFO("Pre grasp success");
        // Visualize
        moveit_msgs::DisplayTrajectory pre_grasp_display_trajectory;
        pre_grasp_display_trajectory.trajectory_start = grasp_plan.pre_grasp.start_state_;
        pre_grasp_display_trajectory.trajectory.push_back(grasp_plan.pre_grasp.trajectory_);
        display_publisher.publish(pre_grasp_display_trajectory);

        // Execute
        move_group->execute(grasp_plan.pre_grasp);

        // Perform grasp if successful
        ROS_INFO("Grasp success");

        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "gripper";
        ocm.header.frame_id = "base_link";
        ocm.orientation = req.Grasp.orientation;
        ocm.absolute_x_axis_tolerance = 1;
        ocm.absolute_y_axis_tolerance = 1;
        ocm.absolute_z_axis_tolerance = 2;
        ocm.weight = 1;

        moveit_msgs::Constraints grasp_constraints;
        grasp_constraints.orientation_constraints.push_back(ocm);
        move_group->setPathConstraints(grasp_constraints);



        robot_state::RobotState start_state(*move_group->getCurrentState());
 
        move_group->setStartState(start_state);

        ROS_WARN("Beginning to plan!");

        const double* j1_ik_start = start_state.getJointPositions("joint_1");
        ROS_WARN("j1 start = %f", *j1_ik_start);

        move_group->setPoseTarget(req.Grasp);
        move_group->setPlanningTime(2);
        res.grasp_plan_success = (move_group->plan(grasp_plan.grasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS); 
        ROS_INFO("Grasp plan: %s", res.grasp_plan_success ? "SUCCESSFUL" : "FAILED");

        if (res.grasp_plan_success)
        {
            // Visualize for debugging
            moveit_msgs::DisplayTrajectory display_trajectory;
            display_trajectory.trajectory_start = grasp_plan.grasp.start_state_;
            display_trajectory.trajectory.push_back(grasp_plan.grasp.trajectory_);
            display_publisher.publish(display_trajectory);
        
            move_group->clearPathConstraints();
        }
        move_group->setPlanningTime(ik_timeout);


    }
    
    return true;
}

bool MoveInterface::executeGrasp(daedalus_msgs::ExecuteGrasp::Request &req,
                                 daedalus_msgs::ExecuteGrasp::Response &res)
{
    if (grasp_plan.planned)
    {
        move_group->asyncExecute(grasp_plan.grasp);
        ros::Duration(req.grasp_time).sleep();
        res.grasp_attempted = grasp("close");
    } else {
        res.grasp_attempted = false;
    }

    grasp_plan.planned = false;
    return true;
}



// This should be named poseCmd and poseCmd should be deleted because
// it was made before I understood quaternions
bool MoveInterface::postureCmd(daedalus_msgs::PostureCmd::Request &req,
                daedalus_msgs::PostureCmd::Response &res) {
    
    geometry_msgs::Pose target_pose;
    target_pose = req.posture;



    move_group->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan target_plan;

    bool success = (move_group->plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Plan status: %s", success ? "SUCCESSFUL" : "FAILED");

    if (success) {
        move_group->execute(target_plan);
    }
    


    res.done = success;
    return true;   

}

bool MoveInterface::positionCmd(daedalus_msgs::PositionCmd::Request &req,
                                daedalus_msgs::PositionCmd::Response &res) 
{

    geometry_msgs::Pose target_pose;

    ros::param::get("/left/pose/pickup/orientation/x", target_pose.orientation.x);
    ros::param::get("/left/pose/pickup/orientation/y", target_pose.orientation.y);
    ros::param::get("/left/pose/pickup/orientation/z", target_pose.orientation.z);
    ros::param::get("/left/pose/pickup/orientation/w", target_pose.orientation.w);

    target_pose.position = req.position;


    move_group->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan target_plan;

    

    bool success = (move_group->plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Plan status: %s", success ? "SUCCESSFUL" : "FAILED");

    if (success) {
        move_group->asyncExecute(target_plan);
    }
    


    res.done = success;
    return true;            
    

}

bool MoveInterface::wait_until_grasp_complete(float grasp_position)
{
    float POSITION_ACCURACY = 0.1;
    int MAX_WAIT = 100;
    
    ros::param::get("stepper_config/goal_position_accuracy", POSITION_ACCURACY);
    ros::param::get("stepper_config/max_goal_time", MAX_WAIT);

    int cnt = 0;

    while (true)
    {
        robot_state::RobotState current_state(*move_group->getCurrentState());

        const double* joint_pos = current_state.getJointPositions("gripper_joint_1");
        if (abs(*joint_pos - grasp_position) < POSITION_ACCURACY)
            return true;

        if (cnt > MAX_WAIT)
            return false;

        cnt++;
        ros::Duration(0.5).sleep();
    }
        


}
        
bool MoveInterface::grasp(std::string pose) {
    std::string param = "grip_pose/" + pose;
    std_msgs::Float32 grasp_position;
    ROS_INFO("Grasp position: %f", grasp_position.data);
    if(ros::param::has(param))
    {
        ros::param::get(param, grasp_position.data);
        graspPub.publish(grasp_position);

        wait_until_grasp_complete(grasp_position.data);

        return true;
    } else {
        ROS_ERROR("Poses parameters not set!");
        return false;
    }

}        

bool MoveInterface::graspCmd(daedalus_msgs::MoveCmd::Request &req,
                             daedalus_msgs::MoveCmd::Response &res)
{
    std::string param = req.pose_name;
    ROS_INFO("Gripping pose: %s", param.c_str());
    bool success = grasp(param);

    

    res.done = success;
    return true;
}

// Return current pose
bool MoveInterface::getPose(daedalus_msgs::GetPos::Request &req,
                            daedalus_msgs::GetPos::Response &res)
{
    geometry_msgs::Pose pose = move_group->getCurrentPose().pose;
    geometry_msgs::Point position = pose.position;
    geometry_msgs::Quaternion orientation = pose.orientation;

    ROS_DEBUG("Position [x: %f y: %f z: %f] Orientation [x: %f y: %f z: %f w: %f]", 
                position.x, position.y, position.z, 
                orientation.x, orientation.y, orientation.z, orientation.w
    );

    res.position = position;
    return true;
}

bool MoveInterface::get_joints(std_srvs::Trigger::Request &req,
                               std_srvs::Trigger::Response & res)
{
    std::vector<std::string> joint_names;
    ros::param::get("stepper_config/joint_names", joint_names);

    robot_state::RobotState current_state(*move_group->getCurrentState());
    for (int i = 0; i < joint_names.size(); i++)
    {
        const double* joint_angle = current_state.getJointPositions(joint_names[i]);
        ROS_INFO("%s: %f", joint_names[i].c_str(), *joint_angle);
    }

    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    MoveInterface interface = MoveInterface(&nh);
    ros::waitForShutdown();
}
