#include <ros/ros.h>
#include <tracker_lib.h>



int main(int argc, char** argv)
{
    // Ros initialization
    ros::init(argc, argv, "object_tracker");
    ros::NodeHandle nh;

    char* pcd_file = argv[1];



    TrackerLib tracker = TrackerLib();
    tracker.setup(&nh, pcd_file);

    int rate_val;

    if(nh.hasParam("/rates/object_tracking")) {
        nh.getParam("/rates/object_tracking", rate_val);
    } else {
        ROS_ERROR("Rate Parameters not loaded.");
        return 1;
    }

    ros::Rate rate(rate_val);

    while (ros::ok())
    {
        tracker.calculate();
        ros::spinOnce();
        rate.sleep();
    }


}