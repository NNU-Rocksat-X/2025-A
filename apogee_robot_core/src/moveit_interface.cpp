/*
* Creates services so other components in the system can move the arm to pre-configured poses

* These services take a string as input which corresponds to a pose param file

* The service then performs the pose

* Author: Aaron Borger <borger.aaron@gmail.com>

*/

#include "moveit_interface.h"

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
    ROS_INFO("q:");
    print_eigen(q);
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
    ROS_INFO("grasp_direction: ");
    print_eigen(grasp_direction);

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


/// @brief - Takes name as request an moves to pre-programmed radian angles for each joint
///          Joint values in yaml file must be in radians
bool MoveInterface::jointPoseCmd(daedalus_msgs::MoveCmd::Request &req,
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
        move_group->execute(target_plan);
        res.done = true;
        return true;
    }
    else {
        res.done = false;
        return false;
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

// Plan Grasp will need to be re-written
// Moveit has cartesian path planning with waypoints, but either it is broken or I cannot get it to work
// Errors involve waypoints not stricly increasing in time and not having a reliable success rate for planning the path.
// Also moveit does not provide any documentation for how to execute the trajectory after planning it.

// Moveit RobotModel also provides a distance function between different states which could be usefull for ensuring the planned path
// is optimized to the shortest path


bool MoveInterface::planGrasp(daedalus_msgs::PlanGrasp::Request &req,
                              daedalus_msgs::PlanGrasp::Response &res) 
{
    // Open gripper before planning TODO: Ensure this does not block as that would slow down planning
    bool g_success = grasp("open");

    geometry_msgs::Pose pre_grasp = calculate_pre_grasp(req.Grasp, 0.2);

    // Plan maneuver to preGrasp position
    move_group->setPoseTarget(pre_grasp);
    GraspPlan grasp_plan;
    res.pre_grasp_success = (move_group->plan(grasp_plan.pre_grasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS); // Plan and check if succeeded
    ROS_INFO("Pre Grasp plan: %s", res.pre_grasp_success ? "SUCCESSFUL" : "FAILED");


    // TODO: Plan preGrasp and grasp simultaneously
    // Current method plans pregrasp then grasp if successfull, this causes the arm to move to preGrasp even if postGrasp isnt possible
    if (res.pre_grasp_success) {
        ROS_INFO("Pre grasp success");
        move_group->execute(grasp_plan.pre_grasp);
        /*
        // An attempt at constraining to the grasp vector. It doesnt work great, but is ok for now I suppose
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "gripper_connector";
        ocm.header.frame_id = "world";
        ocm.orientation = req.preGrasp.orientation;
        ocm.absolute_x_axis_tolerance = 0.1;
        ocm.absolute_y_axis_tolerance = 0.1;
        ocm.absolute_z_axis_tolerance = 0.1;
        ocm.weight = 1.0;

        moveit_msgs::Constraints ee_constraint;
        ee_constraint.orientation_constraints.push_back(ocm);
        move_group->setPathConstraints(ee_constraint);

        robot_state::RobotState start_state(*move_group->getCurrentState());
        move_group->setStartState(start_state);
        

        /* Other constraint method
        planning_interface::MotionPlanRequest motionReq;
        planning_interface::MotionPlanResponse motionRes;
        planning_interface::PlannerManagerPtr planner_interface;
        geometry_msgs::PoseStamped grasp_pose;

        motionReq.group_name = ARM_PLANNING_GROUP;
        grasp_pose.header.frame_id = "base_link";
        std::vector<double> tolerance_pose(3, 0.01);
        std::vector<double> tolerance_angle(3, 0.01);

        grasp_pose.pose = req.Grasp;
        moveit_msgs::Constrains grasp_pose_goal = 
            kinematic_constraints::constructGoalConstraints("gripper_connector", grasp_pose, tolerance_pose, tolerance_angle);

        // Add path constraint
        geometry_msgs::QuaternionStamped constraint_quaternion;
        constraint_quaternion.header.frame_id = "base_link";
        constraint_quaternion.quaternion = req.Grasp.orientation;
        req.path_constraints = kinematic_constraints::constructGoalConstraints("gripper_connector", constraint_quaternion);
        
        // Set bounding box for arm workspace from (-1, 1) for all directions.
        req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -1.0;
        req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z = 1.0;

        planning_interfacce::PlanningContextPtr context = 
        */

        move_group->setPoseTarget(req.Grasp);
        res.grasp_success = (move_group->plan(grasp_plan.grasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS); // Plan and check if succeeded
        ROS_INFO("Grasp plan: %s", res.grasp_success ? "SUCCESSFUL" : "FAILED");

        // Perform grasp if successful
        if (res.grasp_success) {
            ROS_INFO("Grasp success");
            ROS_INFO("Now: %f, grasp: %f", ros::Time::now().toSec(), req.time_to_maneuver.data.toSec());
            while (ros::Time::now() < req.time_to_maneuver.data) {
                // Not a good way of waiting because it blocks
                ros::Duration(0.1).sleep();
            }
            
            // Move to grasp, wait grasp time, then close gripper
            move_group->asyncExecute(grasp_plan.grasp);
            ros::Duration(req.grasp_time).sleep();
            res.gripper_success = grasp("close");
        }

        move_group->clearPathConstraints();

    }
    
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

        
bool MoveInterface::grasp(std::string pose) {
    std::string param = "grip_pose/" + pose;
    std_msgs::Float32 grip_position;
    if(ros::param::has(param))
    {
        ros::param::get(param, grip_position.data);
        graspPub.publish(grip_position);

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

    postureService = nh->advertiseService("posture_cmd", &MoveInterface::postureCmd, this);

    graspService = nh->advertiseService("grasp_cmd", &MoveInterface::graspCmd, this);

    planGraspService = nh->advertiseService("plan_grasp", &MoveInterface::planGrasp, this);
    graspPub = nh->advertise<std_msgs::Float32>("/ARM1/grip_position_cmd", 1);
    graspDetectClient = nh->serviceClient<daedalus_msgs::GraspDetect>("ARM1/is_grasped");

    ros::param::get(ros::this_node::getNamespace() + "/stepper_config/num_joints", num_joints);
    ROS_INFO("Running with %i joints", num_joints);

    pose_param = ros::this_node::getNamespace() + "/pose/";
    //joint_param = ros::this_node::getNamespace() + "/joints/"; commented because there currently is no namespace
    joint_param = "joints/";
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
