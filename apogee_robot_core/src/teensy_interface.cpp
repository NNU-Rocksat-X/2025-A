#include <ros/ros.h>
#include <fstream>
#include <termios.h>
//#include <fcntl.h>

#include <std_msgs/Bool.h>
#include <assert.h>
#include <apogee_robot_core/CommonComms.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <daedalus_msgs/TeensyMsg.h>
#include <math.h>

#include<unistd.h>

#define BUFFER_SIZE (50)

using namespace std;

static_assert(sizeof(CMDPacket) < BUFFER_SIZE, "Buffer too small.");


CMDPacket tx;
RESPacket rx;

uint8_t seq;
std::vector<double> deg_per_step;
std::vector<double> deg_per_enc_step;
std::vector<double> rad_per_enc_step;

double rad_to_deg(double rad)
{
    return rad * 180/M_PI;
}

double deg_to_rad(double deg)
{
    return deg * M_PI/180;
}

void led_cb(const std_msgs::Bool::ConstPtr& msg)
{
    tx.led = msg->data; // note the different data typess
}

void positionCB(const daedalus_msgs::TeensyMsg::ConstPtr& msg)
{
    cout << "Position Callback----------------------------------------------------" << endl;

    //sleep(1000);
    //ROS_INFO("Velocity-------------");
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        // teensy msg gives radians/sec
        // CMDPacket needs cmd in terms of steps/ms
        double deg_sec = msg->steps[i];
        double deg_ms = deg_sec;
        int step_ms = (int)(deg_ms / deg_per_enc_step[i]);
        tx.joint_velocity_cmd[i] = int(deg_ms);
        //ROS_INFO("deg/sec: %f", deg_sec);
        //ROS_INFO("deg/ms: %f", deg_ms);
        //ROS_INFO("J%i: %i", i, step_ms);
    }
}


double adjust_angle(double angle) {
    double two_pi = 2 * std::acos(-1.0);
    while (angle < -std::acos(-1.0)) {
        angle += two_pi;
    }
    while (angle > std::acos(-1.0)) {
        angle -= two_pi;
    }
    return angle;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teensy_interface");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/led", 1, led_cb);
    ros::Subscriber ctrl_sub = nh.subscribe("joint_position_cmd", 2, &positionCB);
    ros::Publisher robotStatePub = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 2);

    moveit_msgs::DisplayRobotState robotState;

    std::vector<std::string> joint_names;
    if (ros::param::has("/stepper_config/joint_names")) {
        ros::param::get("/stepper_config/joint_names", joint_names);
        ros::param::get("/stepper_config/deg_per_step", deg_per_step);
        ros::param::get("/stepper_config/deg_per_enc_step", deg_per_enc_step);
        ros::param::get("/stepper_config/rad_per_enc_step", rad_per_enc_step);
    } else {
        ROS_ERROR("Stepper config param not loaded!");
    }

    // Initialize robot state
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        robotState.state.joint_state.name.push_back(joint_names[i].c_str());
    }
    robotState.state.joint_state.position.resize(NUM_JOINTS);
    robotState.state.joint_state.velocity.resize(NUM_JOINTS);
    robotState.state.joint_state.effort.resize(NUM_JOINTS);

    ofstream teensy_tx;
    ifstream teensy_rx;

    cout << "Opened" << endl;

    ros::Rate rate(50);


    //ROS_INFO("Size: %i", sizeof(CMDPacket));

    
    while(ros::ok())
    {
        tx.seq = seq;
        tx.FRAME_SYNC_LSB = 0x55;
        tx.FRAME_SYNC_MSB = 0x70;
        seq += 1;
        if (seq == 254)
        {
            seq = 0;
        }

        //cout << "Start" << endl;
    
        tx.crc = crc16((unsigned char*)&tx, (int)sizeof(CMDPacket) - 4); // Subtract 2 so the crc is not calculated over the crc
        teensy_tx.open("/dev/ttyACM0");                           // Set Serial port here
        teensy_tx.write((char*)&tx, sizeof(CMDPacket));
        teensy_tx.close();

        /*for (int ii = 0; ii < NUM_JOINTS; ++ii) {
            cout << tx.joint_velocity_cmd[ii] << endl;// output the output to the teensy
            
        }*/

        char buffer[BUFFER_SIZE];
        teensy_rx.open("/dev/ttyACM0"); 
        teensy_rx.read((char*)&rx, sizeof(RESPacket));

        ROS_INFO("TX SEQ: %u RX SEQ: %u STATUS: %u", seq, rx.seq, rx.reserved);
        teensy_rx.close();

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            // Robotstate is in radians
            robotState.state.joint_state.velocity[i] = 0;
            robotState.state.joint_state.effort[i] = 0;
            // TODO: add adjust angle, but it causes problems
            robotState.state.joint_state.position[i] = rx.joint_step_position[i] * rad_per_enc_step[i];
            //ROS_INFO("J%i: %f", i, robotState.state.joint_state.position[i]);
        }
        ROS_INFO("publishing");
        robotStatePub.publish(robotState);

        ros::spinOnce();
        rate.sleep();
    }

}