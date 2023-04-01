#include <ros/ros.h>
#include "apogee_vision/TrajectorySearch.h"

#define NUM_ARGS (12)

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_search_test");
    ros::NodeHandle nh;

    ROS_INFO("args: %i", argc);
    if (argc != NUM_ARGS + 1)
    {
        ROS_WARN("Enter the full state as the args.");
    }

    float args[NUM_ARGS];

    for (int i = 0; i < NUM_ARGS; i++)
    {
        args[i] = std::atof(argv[i+1]);
    }

    Vector3f pos(args[0], args[1], args[2]);
    Vector3f vel(args[3], args[4], args[5]);
    Vector3f acc(args[6], args[7], args[8]);
    Vector3f robot_base(args[9], args[10], args[11]);

    Searcher searcher(pos, vel, acc);

    
    float norm_time;
    Vector3f predicted_position = searcher.solve(0.5, norm_time);
    ROS_INFO("Delta closest pass: %f", norm_time);

}