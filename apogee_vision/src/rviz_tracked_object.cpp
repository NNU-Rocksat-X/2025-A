#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose object_pose_;

void wrench_pose_cb(const geometry_msgs::Pose& msg)
{
    object_pose_ = msg;
    ROS_INFO("Recieved pose!");
}

// Args: Mesh file path, visualization topic, pose topic
int main(int argc, char** argv) {

  ros::init(argc, argv, "display_object");

  ros::NodeHandle nh;
  ros::Rate rate(50);

  ros::Publisher object_pub  = nh.advertise<visualization_msgs::Marker>(argv[2], 10);
  ros::Subscriber sub = nh.subscribe(argv[3], 1, wrench_pose_cb);

  while (ros::ok())
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "display_object";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    //marker.type = marker.MESH_RESOURCE;
    marker.mesh_resource = argv[1];
    marker.mesh_use_embedded_materials = true;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = object_pose_;


    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    object_pub.publish(marker);

    ROS_INFO("Publishing...");

    ros::spinOnce();
    rate.sleep();

  }

}
