#include <ros/ros.h>
#include <fstream>

#include <std_msgs/Bool.h>
#include <assert.h>
#include <apogee_robot_core/CommonComms.h>

#define BUFFER_SIZE (50)

using namespace std;


/*

*/
static_assert(sizeof(CMDPacket) < BUFFER_SIZE, "Buffer too small.");

CMDPacket tx;
RESPacket rx;

void led_cb(const std_msgs::Bool::ConstPtr& msg)
{
    tx.led = msg->data; // note the different data typess
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teensy_interface");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/led", 1, led_cb);

    ofstream teensy_tx;
    ifstream teensy_rx;

    cout << "Opened" << endl;

    tx.joint_velocity_cmd[1] = 1.5;

    ros::Rate rate(50);


    //ROS_INFO("Size: %i", sizeof(CMDPacket));

    
    while(ros::ok())
    {
        tx.crc = crc16((unsigned char*)&tx, sizeof(CMDPacket) - 2); // Subtract 2 so the crc is not calculated over the crc
        teensy_tx.open("/dev/ttyACM0");
        teensy_tx.write((char*)&tx, sizeof(CMDPacket));
        teensy_tx.close();

        char buffer[BUFFER_SIZE];
        teensy_rx.open("/dev/ttyACM0");
        teensy_rx.read((char*)&rx, sizeof(RESPacket));

        teensy_rx.close();

        ros::spinOnce();
        rate.sleep();
    }

}