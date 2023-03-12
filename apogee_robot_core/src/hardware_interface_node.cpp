#include <controller_manager/controller_manager.h>
#include <apogee_hardware_interface.h>

typedef Arm2D2Interface MarshaArm;

#define RATE (50.0)

int main(int argc, char** argv)
{
    
    ROS_INFO("Starting node");
    ros::init(argc, argv, "marsha_hardware_interface");

    ros::NodeHandle nh;

    
    MarshaArm arm(nh);

    
    controller_manager::ControllerManager cm(&arm);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(RATE);


    while(ros::ok()) {
        ros::Time time = ros::Time::now();
        ros::Duration period = time - prev_time;

        arm.read();

        cm.update(time, period);


        arm.write();

        rate.sleep();
    }

    
    return 0;
}