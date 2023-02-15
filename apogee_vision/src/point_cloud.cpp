#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <daedalus_msgs/RGBDFrame.h>
#include <math.h>

#define FOVY (45)

typedef Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Frame;

struct CamProperties {
    bool initialized;
    int posX;
    int posY;
    int height;
    int width;
    int fovX;
    int fovY;
    float focalX;
    float focalY;
} cam_;

ros::Publisher pc_pub;
ros::Publisher filtered_pc_pub;
int seq_ = 0;
std_msgs::UInt8MultiArray mask_arr;
bool mask_recieved = false;

uint8_t get_mask(int r, int c)
{
    return mask_arr.data[mask_arr.layout.dim[1].stride*r + c];
}

void initialize_cam(CamProperties &cam, int height, int width)
{
    cam.posX = 0;
    cam.posY = 0;
    cam.height = height;
    cam.width = width;
    cam.fovY = FOVY;
    cam.fovX = (int)(cam.fovY * (width/height));
    cam.focalX = (width/2) / tan(cam.fovX * M_PI/180);
    cam.focalY = (height/2) / tan(cam.fovY * M_PI/180);
    cam.initialized = true;
}

// C++ doesn't let you copy it with '=' or memcpy, so we have to copy
// the msg like this to use it out of the callback function
void maskCB(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    mask_arr.data.clear(); // Not clearing the previous mask causes big issues
    mask_arr.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mask_arr.layout.dim[0] = msg->layout.dim[0];
    mask_arr.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mask_arr.layout.dim[1] = msg->layout.dim[1];    
    for (int i = 0; i < msg->layout.dim[0].stride; i++)
    {
        mask_arr.data.push_back(msg->data[i]);
    }
    mask_recieved = true;
    ROS_INFO("Mask Recieved!");
    
}


void depthCB(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (!cam_.initialized) {
        initialize_cam(cam_, msg->layout.dim[0].size, msg->layout.dim[1].size);
    }

    std::vector<float> d_data = msg->data;
    Frame d_frame(d_data.data(), cam_.height, cam_.width);
    
    float min = d_frame.minCoeff();
    float max = d_frame.maxCoeff();

    // Scale between 0 and 1
    d_frame = (1 - (d_frame.array() - min) / (max - min)).matrix();

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

    for (int c = 0; c < cam_.width; c++)
    {
        for (int r = 0; r < cam_.height; r++)
        {
            pcl::PointXYZ p;
            p.z = d_frame(r, c);
            p.x = (c - cam_.posX) * p.z / cam_.focalX;
            p.y = (r - cam_.posY) * p.z / cam_.focalY;

            // Only add point if it is within the mask
            if (mask_recieved){
                uint8_t mask_val = get_mask(r, c);
                if (get_mask(r, c) > 0)
                {
                    filteredCloudPtr->points.push_back(p);
                }
            }
            // If no mask has been recieved just add all points
            cloudPtr->points.push_back(p);

        }
    }

    // Convert PointXYZ to pcl::PointCloud2
    pcl::PCLPointCloud2::Ptr filtered_pcl_pc2(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*filteredCloudPtr, *filtered_pcl_pc2);

    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloudPtr, *pcl_pc2);

    // Reduce number of points for performance
    /*
    pcl::PCLPointCloud2::Ptr filtered_cloud(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(pcl_pc2);
    sor.setLeafSize(0.0001f, 0.0001f, 0.0001f);
    sor.filter(*filtered_cloud);
    */


    // Convert pcl::PointCloud2 to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 pc2;
    pcl_conversions::fromPCL(*pcl_pc2, pc2);

    sensor_msgs::PointCloud2 filtered_pc2;
    pcl_conversions::fromPCL(*filtered_pcl_pc2, filtered_pc2);

    pc2.header.seq = seq_;
    pc2.header.frame_id = "world";
    pc2.header.stamp = ros::Time::now();

    filtered_pc2.header.seq = seq_;
    filtered_pc2.header.frame_id = "world";
    filtered_pc2.header.stamp = ros::Time::now();

    pc_pub.publish(pc2);
    filtered_pc_pub.publish(filtered_pc2);
    seq_++;

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud");
    ros::NodeHandle nh;
    cam_.initialized = false;

    ros::Subscriber mask_sub = nh.subscribe("/object_mask", 1, &maskCB);
    ros::Subscriber frame_sub = nh.subscribe("/depth", 1, &depthCB);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 10);
    filtered_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 10);

    ros::spin();
}