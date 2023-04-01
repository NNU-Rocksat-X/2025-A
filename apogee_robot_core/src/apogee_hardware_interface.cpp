#include <apogee_hardware_interface.h>

// Rate is used to interfpolate trajectories
Arm2D2Interface::Arm2D2Interface(ros::NodeHandle &nh_) {
    nh = nh_;
    
    try {
        ros::param::get("stepper_config/num_joints", num_joints);
        ros::param::get("stepper_config/num_grip_joints", num_grip_joints);
        ros::param::get("stepper_config/joint_names", joint_names);
        ros::param::get("stepper_config/deg_per_step", deg_per_steps);

        for (int i = 0; i < num_joints; i++)
        {
            std::map<std::string, float> gain;
            ros::param::get("stepper_config/gains/"+joint_names[i], gain);
            gains.push_back(gain);
        }
    }
    catch (int e) {
        ROS_ERROR("Stepper Config Parameters Not Loaded!");
    }

    step_pub = nh.advertise<daedalus_msgs::TeensyMsg>("joint_position_cmd", 10);
    enc_sub = nh.subscribe("display_robot_state", 100, &Arm2D2Interface::encoderCallBack, this);
    trajectory_goal_sub = nh.subscribe("arm2d2/controllers/position/follow_joint_trajectory/goal", 100, 
                                        &Arm2D2Interface::trajectory_cb, this);

    // The commands from the control handle are not actually followed 
    // because they don't follow the trajectory
    
    for (int i = 0; i < num_joints; i++) {
        ROS_INFO("Initiallizing joint: %s", joint_names[i].c_str());
        hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
        joint_state_interface.registerHandle(state_handle);


        hardware_interface::JointHandle control_handle(state_handle, &fake_cmd[i]);
        joint_control_interface.registerHandle(control_handle);

    }
    

    registerInterface(&joint_state_interface);
    registerInterface(&joint_control_interface);

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




// Returns velocity
double Arm2D2Interface::pid_controller(int joint_id, double position_cmd)
{
    
    static double error_sum[15];
    static std::vector<double> prev_error[15];
    static std::vector<double> prev_time[15];
    static int d_cnt = 0;
    // P component
    double error = position_cmd - pos[joint_id];
    double v = error * gains[joint_id]["p"];

    // I component
    error_sum[joint_id] += error * gains[joint_id]["i"];
    // Clegg Integrator (if close to zero reset integral)
    if (error < 0.001 && error > -0.001)
    {
        error_sum[joint_id] = 0;
    }
    if (error_sum[joint_id] > gains[joint_id]["i_clamp"]) 
    {
        error_sum[joint_id] = gains[joint_id]["i_clamp"];
    }
    else if (error_sum[joint_id] < -1*gains[joint_id]["i_clamp"])
    {
        error_sum[joint_id] = -1*gains[joint_id]["i_clamp"];
    }
    //ROS_INFO("J:%i sum: %f", joint_id, error_sum[joint_id]);
    v += error_sum[joint_id];

    // D Component
    double now = ros::Time::now().toSec();

    double d = 0;
    if (prev_error[joint_id].size() > 0)
    {
        d = (error - prev_error[joint_id].back()) / (now - prev_time[joint_id].back());
    }

    prev_error[joint_id].push_back(error);
    prev_time[joint_id].push_back(now);

    if (prev_error[joint_id].size() > 10)
    {
        prev_error[joint_id].pop_back();
        prev_time[joint_id].pop_back();
    }
    v += d * gains[joint_id]["d"];
    
    return v;

}

// Checks if it is time to go to next step in trajectory
// Updates cmd if it is
void Arm2D2Interface::update_setpoint()
{
    double now = ros::Time::now().toSec();
    if (current_trajectory_point < (int)position_setpoints.size())
    {
        // If time for next trajectory point
        if (now > setpoint_times[current_trajectory_point])
        {
            ROS_DEBUG("Following trajectory %i/%i - Times: %i.", current_trajectory_point, (int)position_setpoints.size(), (int)setpoint_times.size());
            current_trajectory_point++;
        }
    }

    if (current_trajectory_point < (int)position_setpoints.size())
    {
        // Add the interpolated setpoint for each joint to cmd
        for (int i = 0; i < num_joints - num_grip_joints; i++)
        {
            std::string joint_name = joint_names[i];
            double next_pos = position_setpoints[current_trajectory_point][joint_name];
            double next_time = setpoint_times[current_trajectory_point];
            double prev_pos = position_setpoints[current_trajectory_point-1][joint_name];
            double prev_time = setpoint_times[current_trajectory_point-1];
            if (next_time - prev_time == 0)
            {
                ROS_ERROR("Divid by 0 during interpolation");
            }
            cmd[i] = prev_pos + (now - prev_time) * (next_pos - prev_pos) / (next_time - prev_time);
        }
    }

}

void Arm2D2Interface::write() 
{
    static int update_cnt = 0;
    if (update_cnt > 1)
    {
        update_setpoint();
        update_cnt = 0;
    } else {
        update_cnt++;
    }
    daedalus_msgs::TeensyMsg msg;

    for(int i = 0; i < num_joints-num_grip_joints; i++) {  
        //ROS_INFO("J%i - %f", i, cmd[i]);    
        msg.steps.push_back(pid_controller(i, cmd[i]));
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

void Arm2D2Interface::trajectory_cb(const control_msgs::FollowJointTrajectoryActionGoal &msg)
{
    position_setpoints.clear();
    setpoint_times.clear();
    current_trajectory_point = 0; // start at one because point 0 is the beginning state
    int trajectory_size = msg.goal.trajectory.points.size();
    double start_time = msg.header.stamp.toSec();
    for (int i = 0; i < trajectory_size; i++)
    {
        std::map<std::string, double> position_sp;
        if ((int)msg.goal.trajectory.joint_names.size() != num_joints - num_grip_joints)
        {
            ROS_ERROR("Trajectory Joint Missmatch - Trajectory Joint Size: %i, num_joints: %i", (int)msg.goal.trajectory.joint_names.size(), num_joints-num_grip_joints);
        }
        for (int j = 0; j < msg.goal.trajectory.joint_names.size(); j++)
        {
            position_sp[msg.goal.trajectory.joint_names[j]] = msg.goal.trajectory.points[i].positions[j];
        }
        position_setpoints.push_back(position_sp);
        setpoint_times.push_back(start_time + msg.goal.trajectory.points[i].time_from_start.toSec() - HEADSTART);
    }

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