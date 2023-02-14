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
int seq_ = 0;

// Access according to 
// http://docs.ros.org/en/melodic/api/std_msgs/html/msg/MultiArrayLayout.html
class ColorFrame
{
    public:
    std_msgs::UInt8MultiArray* arr;
    ColorFrame(std_msgs::UInt8MultiArray* _arr) : arr{_arr} {}

    uint8_t access(int i, int j, int k)
    {
        return arr->data[arr->layout.dim[1].stride*i + arr->layout.dim[2].stride*j + k];
    }
};


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

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    for (int c = 0; c < cam_.width; c++)
    {
        for (int r = 0; r < cam_.height; r++)
        {
            pcl::PointXYZ p;
            p.z = d_frame(r, c);
            p.x = (c - cam_.posX) * p.z / cam_.focalX;
            p.y = (r - cam_.posY) * p.z / cam_.focalY;
            cloudPtr->points.push_back(p);
        }
    }

    // Convert PointXYZ to pcl::PointCloud2
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloudPtr, *pcl_pc2);

    // Reduce number of points for performance
    pcl::PCLPointCloud2::Ptr filtered_cloud(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(pcl_pc2);
    sor.setLeafSize(0.001f, 0.001f, 0.001f);
    sor.filter(*filtered_cloud);


    // Convert pcl::PointCloud2 to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 pc2;
    pcl_conversions::fromPCL(*filtered_cloud, pc2);

    pc2.header.seq = seq_;
    pc2.header.frame_id = "world";
    pc2.header.stamp = ros::Time::now();

    pc_pub.publish(pc2);
    seq_++;

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud");
    ros::NodeHandle nh;
    cam_.initialized = false;

    ros::Subscriber frame_sub = nh.subscribe("/depth", 1, &depthCB);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 10);

    ros::spin();
}