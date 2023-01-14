#include <ctrl_interface.h>

///////////////////////////////////////////////////////
///                 GLOBAL VARIABLES
/////////////////////////////////////////////////////// 
extern mjModel* m;
extern mjData* d;

///////////////////////////////////////////////////////
///                 STATIC VARIABLES
/////////////////////////////////////////////////////// 

static float vel_cmd[MAX_JOINTS];
static int num_joints = 0;
static int num_grip_joints = 0;
static int total_joints = 0;
static float grip_cmd = 0;
static float grip_kv = 5;

static ros::Subscriber ctrl_sub;
static ros::Subscriber grip_ctrl_sub;
static ros::Publisher rgb_pub;
static ros::Publisher d_pub;
static ros::Publisher cam_pose_pub;
static ros::Publisher robotStatePub;
static ros::Publisher ObjectPosePub;
static ros::Publisher ObjectTwistPub;
static ros::Publisher ObjectTwistAccelPub;
static ros::ServiceServer applyForceService;
static ros::ServiceServer graspDetectService;
static ros::ServiceServer resetService;
static ros::ServiceServer setBodyService;

static std_msgs::UInt8MultiArray rgbArr;
static std_msgs::Float32MultiArray dArr;
static geometry_msgs::Pose cam_pose;

static moveit_msgs::DisplayRobotState robotState;

static bool reset_flag = false;

///////////////////////////////////////////////////////
///                 STATIC FUNCTIONS
///////////////////////////////////////////////////////  

static float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

///////////////////////////////////////////////////////
///                 GLOBAL FUNCTIONS
/////////////////////////////////////////////////////// 



extern bool apply_force_cb(mujoco_ros::ApplyForce::Request &req,
                           mujoco_ros::ApplyForce::Response &res);

extern void get_model_pose(geometry_msgs::Pose* pose, int model_id);

void positionCB(const daedalus_msgs::TeensyMsg::ConstPtr& msg)
{
    for (int i = 0; i < num_joints; i++)
    {
        vel_cmd[i] = msg->steps[i];
    }
}

void gripCB(const std_msgs::Float32::ConstPtr& msg)
{
    grip_cmd = msg->data;
}

bool grasp_detect_cb(daedalus_msgs::GraspDetect::Request &req,
                    daedalus_msgs::GraspDetect::Response &res)
{
    // If three gripper links are in contact with object then it's grasped
    // Greater than is also grasped because the object can be in contact with the palm
    if (d->ncon >= 3)
    {
        res.is_grasped = true;
    } else {
        res.is_grasped = false;
    }
    return true;
}

void ros_reset(void)
{
    if (reset_flag)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
        reset_flag = false;
    }
}

bool reset_cb(std_srvs::Trigger::Request &req,
              std_srvs::Trigger::Response &res)
{
    reset_flag = true;
    res.success = true;
    return true;
}

bool set_body_cb(mujoco_ros::SetBody::Request &req,
                 mujoco_ros::SetBody::Response &res)
{
    int body_id = mj_name2id(m, mjOBJ_BODY, req.body_name.c_str());
    set_body(body_id, req.pose.x, req.pose.y, req.pose.z);

    if (body_id > 0) {
        res.success = true;
    } else {
        res.success = false;
    }

    return true;
}

// Called at top of main
void init_ros_node()
{
    static ros::NodeHandle nh;
    ctrl_sub = nh.subscribe("ARM1/joint_position_cmd", 2, &positionCB);
    grip_ctrl_sub = nh.subscribe("ARM1/grip_position_cmd", 2, &gripCB);
    
    rgb_pub = nh.advertise<std_msgs::UInt8MultiArray>("/frame", 1);
    d_pub = nh.advertise<std_msgs::Float32MultiArray>("/depth", 1);
    cam_pose_pub = nh.advertise<geometry_msgs::Pose>("/cam_pose", 1);
    robotStatePub = nh.advertise<moveit_msgs::DisplayRobotState>("ARM1/display_robot_state", 2);
    ObjectPosePub = nh.advertise<geometry_msgs::Pose>("mujoco/object_pose", 1);
    ObjectTwistPub = nh.advertise<geometry_msgs::Twist>("/mujoco/object_twist", 1);
    ObjectTwistAccelPub = nh.advertise<geometry_msgs::Twist>("/mujoco/object_accel_twist", 1);
    applyForceService = nh.advertiseService("mujoco/apply_force", apply_force_cb);
    resetService = nh.advertiseService("/mujoco/reset", reset_cb);
    setBodyService = nh.advertiseService("/mujoco/set_body", set_body_cb);
    graspDetectService = nh.advertiseService("ARM1/is_grasped", grasp_detect_cb);

    // Get ros parameters
    if (nh.hasParam("stepper_config/num_sim_joints"))
    {
        nh.getParam("stepper_config/num_sim_joints", num_joints);
    }

    if (nh.hasParam("stepper_config/num_grip_joints"))
    {
        nh.getParam("stepper_config/num_grip_joints", num_grip_joints);
    }

    total_joints = num_joints + num_grip_joints;

    for (int i = 0; i < num_joints; i++)
    {
        vel_cmd[i] = 0;
    }
    ROS_INFO("Ros Initialized!");
}

// mjcb_control callback
void ctrl_robot(const mjModel* m, mjData* d)
{
    
    ros::spinOnce();
    //ROS_INFO("J1: %f, J2: %f, J3: %f, J4: %f, J5: %f", d->sensordata[0], d->sensordata[1], d->sensordata[2], d->sensordata[3], d->sensordata[4]);
    for (int i = 0; i < num_joints; i++)
    {
        
        d->ctrl[i] = vel_cmd[i];

        // Update Robot State
        robotState.state.joint_state.position[i] = d->sensordata[i];
        robotState.state.joint_state.velocity[i] = (double)vel_cmd[i];
        robotState.state.joint_state.effort[i] = 0;
    }
    
    float grip_v_cmd;
    for (int i = 0; i < num_grip_joints; i++)
    {
        int grip_i = num_joints+i;
        robotState.state.joint_state.position[grip_i] = d->sensordata[grip_i];

        grip_v_cmd = grip_kv * (grip_cmd - d->sensordata[grip_i]);

        d->ctrl[grip_i] = grip_v_cmd;
        robotState.state.joint_state.velocity[grip_i] = 0;
        robotState.state.joint_state.effort[grip_i] = 0;
    }
    
    robotStatePub.publish(robotState);
}

void init_robot_state(void)
{

    robotState.state.joint_state.name.push_back("joint_1");
    robotState.state.joint_state.name.push_back("joint_2");
    robotState.state.joint_state.name.push_back("joint_3");
    robotState.state.joint_state.name.push_back("joint_4");
    robotState.state.joint_state.name.push_back("joint_5");
    robotState.state.joint_state.name.push_back("gripper_joint");
    robotState.state.joint_state.name.push_back("actuator_joint_1");
    robotState.state.joint_state.name.push_back("actuator_joint_2");
    robotState.state.joint_state.name.push_back("actuator_joint_3");

    robotState.state.joint_state.position.resize(total_joints);
    robotState.state.joint_state.velocity.resize(total_joints);
    robotState.state.joint_state.effort.resize(total_joints);
}

void init_frame_pub(int w, int h)
{
    rgbArr.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rgbArr.layout.dim[0].label = "height";
    rgbArr.layout.dim[0].size = h;
    rgbArr.layout.dim[0].stride = 3*w*h;
    rgbArr.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rgbArr.layout.dim[1].label = "width";
    rgbArr.layout.dim[1].stride = 3*w;
    rgbArr.layout.dim[1].size = w;
    rgbArr.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rgbArr.layout.dim[2].label = "channel";
    rgbArr.layout.dim[2].size = 3;
    rgbArr.layout.dim[2].stride = 3;

    for (int i = 0; i < 3*w*h; i++)
    {
        rgbArr.data.push_back(0);
    }

    dArr.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dArr.layout.dim[0].label = "height";
    dArr.layout.dim[0].size = h;
    dArr.layout.dim[0].stride = w*h;
    dArr.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dArr.layout.dim[1].label = "width";
    dArr.layout.dim[1].stride = w;
    dArr.layout.dim[1].size = w;

    for (int i = 0; i < w*h; i++)
    {
        dArr.data.push_back(0);
    }

}

void send_rgb(unsigned char* rgb)
{
    for (int i = 0; i < int(rgbArr.layout.dim[0].stride); i++)
    {
        rgbArr.data[i] = rgb[i];
    }

    rgb_pub.publish(rgbArr);
}

void send_depth(float* depth)
{
    for (int i = 0; i < dArr.layout.dim[0].stride; i++)
    {
        dArr.data[i] = depth[i];
    }
    d_pub.publish(dArr);
}

void send_cam_pos(double* cam_pos, double* cam_quat)
{
    //cam_pose.orientation.x = cam_quat[0];
    cam_pose.orientation.y = cam_quat[1];
    //cam_pose.orientation.z = cam_quat[2];
    cam_pose.orientation.w = cam_quat[3];

    cam_pose_pub.publish(cam_pose);
}

void get_cam_pos(double* lookat, double dist, double azimuth, double elevation)
{
    cam_pose.position.z = lookat[2] + (-1*dist * sin(elevation * PI/180));
    double Dxy = cam_pose.position.z / tan(elevation * PI/180);
    cam_pose.position.x = lookat[0] + Dxy * cos(azimuth * PI/180);
    cam_pose.position.y = lookat[1] + Dxy * sin(azimuth * PI/180);

    cam_pose.orientation.x = azimuth;
    cam_pose.orientation.z = -1*elevation;
}

// This should be refactored to allow for tracking multiple objects
// Mujoco lib offers mj_name2id which may be useful
void send_object_state()
{
    geometry_msgs::Pose pose;
    get_model_pose(&pose, 1);
    ObjectPosePub.publish(pose);

    geometry_msgs::Twist twist;
    get_model_twist(&twist, 1);
    ObjectTwistPub.publish(twist);

    get_model_twist_accel(&twist, 1);
    ObjectTwistAccelPub.publish(twist);
}