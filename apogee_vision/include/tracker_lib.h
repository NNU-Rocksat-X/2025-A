#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/distance_coherence.h>

#include <assert.h> // Testing only

using namespace pcl::tracking;

typedef pcl::PointXYZ RefPointType;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter ;

class TrackerLib
{
    private:
        ros::Publisher pose_pub;
        ros::Subscriber point_cloud_sub;

        CloudPtr cloud_pass_;
        CloudPtr cloud_pass_downsampled_;
        CloudPtr target_cloud;

        geometry_msgs::Pose object_pose_;
        Eigen::Affine3f initial_transformation_;

        KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> tracker_;
        double downsampling_grid_size_;
        int counter;
        bool new_cloud_;

        void publishResult(void);

        void cloud_cb (const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud_msg);
    public:
        TrackerLib();
        ~TrackerLib();

        /* @Brief - Initializes parameters and loads reference file
        *           Call immediately after constructor
        *
        * @param[in] pose_pub - Ros publisher to publish object pose
        *
        * @param[in] pcd_file - char array with path to reference pcd file
        */
        void setup(ros::NodeHandle* nh, char* pcd_file);

        void calculate(void);


        // --------- GUI Only -------------
        CloudConstPtr get_reference(void);
        uint8_t get_recv_cloud(CloudPtr cloud);
        void get_transformation(Eigen::Affine3f &transformation);
        ParticleFilter::PointCloudStatePtr get_particles();
        // --------------------------------

};