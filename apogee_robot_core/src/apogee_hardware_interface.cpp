#include <apogee_hardware_interface.h>

Arm2D2Interface::Arm2D2Interface(ros::NodeHandle &nh_) {
    nh = nh_;
    
    try {
        ros::param::get("stepper_config/num_joints", num_joints);
        ros::param::get("stepper_config/joint_names", joint_names);
        ros::param::get("stepper_config/deg_per_step", deg_per_steps);
    }
    catch (int e) {
        ROS_ERROR("Stepper Config Parameters Not Loaded!");
    }

    step_pub = nh.advertise<daedalus_msgs::TeensyMsg>("joint_position_cmd", 10);

    enc_sub = nh.subscribe("display_robot_state", 100, &Arm2D2Interface::encoderCallBack, this);

    for (int i = 0; i < num_joints; i++) { // Dont initialize fake dof here
        ROS_INFO("Initiallizing joint: %s", joint_names[i].c_str());
        hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
        joint_state_interface.registerHandle(state_handle);


        hardware_interface::JointHandle velocity_handle(state_handle, &cmd[i]);
        joint_velocity_interface.registerHandle(velocity_handle);

    }
    
    /*
    int grip_id = num_joints - 1;
    ROS_DEBUG("gripper name: %s", joint_names[grip_id].c_str());
    hardware_interface::JointStateHandle gripper_state_handle(joint_names[grip_id], &pos[grip_id], &vel[grip_id], &eff[grip_id]);
    joint_state_interface.registerHandle(gripper_state_handle);
    hardware_interface::JointHandle gripper_effort_handle(gripper_state_handle, &cmd[grip_id]);
    joint_effort_interface.registerHandle(gripper_effort_handle);
    */

    registerInterface(&joint_state_interface);
    registerInterface(&joint_velocity_interface);

    // Initialize values
    for (int i = 0; i < num_joints; i++) {
        cmd[i] = 0.0;
        pos[i] = 0.0;
        vel[i] = 0.0;
        eff[i] = 0.0;
    }



}

void Arm2D2Interface::read() {

}

void Arm2D2Interface::write() {

    //ROS_INFO("Writing %f, %f, %f, %f, %f", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

    daedalus_msgs::TeensyMsg msg;

    for(int i = 0; i < num_joints; i++) {
        msg.steps.push_back(cmd[i]);
    }

    step_pub.publish(msg);
}

void Arm2D2Interface::encoderCallBack(const moveit_msgs::DisplayRobotState &msg) {
    for(int i = 0; i < num_joints; i++) {
        pos[i] = msg.state.joint_state.position[i];
        vel[i] = msg.state.joint_state.velocity[i];
        eff[i] = 0.0;
    }

}

void Arm2D2Interface::update_time(ros::Time time)
{
    // Note: This should have a semaphor, 
    // but as this part is used to just fake an extra dof it's not worth it
    previous_time = current_time;
    current_time = time;
}

////////////////////////////////////////////////////////////
///                    STATIC FUNCTIONS
/////////////////////////////////////////////////////////////

double Arm2D2Interface::radToDeg(double rad) {
	return rad / M_PI * 180.0;
}

double Arm2D2Interface::degToRad(double deg) {
    return deg / 180.0 * M_PI;
}

Arm2D2Interface::~Arm2D2Interface() {}