// Remove after testing
#include <tracker_lib.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>
#include <math.h>

TrackerLib tracker_;
using namespace std::chrono_literals;


bool drawParticles (pcl::visualization::PCLVisualizer& viz)
{
    ParticleFilter::PointCloudStatePtr particles = tracker_.get_particles();
    
    if (particles)
    {
        // Set pointcloud with particle's points
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        for (const auto& particle: *particles)
        {
            pcl::PointXYZ point;
            point.x = particle.x;
            point.y = particle.y;
            point.z = particle.z;
            particle_cloud->push_back (point);
        }

        
        // Draw red particles
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);

        if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
        {
            viz.addPointCloud (particle_cloud, red_color, "particle cloud");
        }
        return true;
    } else {
        return false;
    }
}

void drawResult (pcl::visualization::PCLVisualizer& viz)
{
    
    Eigen::Affine3f transformation;
    tracker_.get_transformation(transformation);

    transformation.translation() += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
    CloudPtr result_cloud (new Cloud());
    pcl::transformPointCloud<RefPointType> (*(tracker_.get_reference()), *result_cloud, transformation);

    pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);

    
    if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
    {
        viz.addPointCloud(result_cloud, blue_color, "resultcloud");
        //viz.resetCameraViewpoint("resultcloud");
    }
    
    
}

void viz_cb (pcl::visualization::PCLVisualizer& viz)
{
    tracker_.calculate();
    CloudPtr recv_cloud (new Cloud());
    if (tracker_.get_recv_cloud(recv_cloud))
    {
        std::this_thread::sleep_for(1s);
    }
    if (!viz.updatePointCloud (recv_cloud, "cloudpass"))
    {
        viz.addPointCloud(recv_cloud, "cloudpass");
        //viz.resetCameraViewpoint("cloudpass");
    }
    
    bool ret = drawParticles (viz);
    if (ret)
    {
        drawResult(viz);
    }
}


int main(int argc, char** argv)
{
    // Ros initialization
    ros::init(argc, argv, "object_tracker");
    ros::NodeHandle nh;

    char* pcd_file = argv[1];

    tracker_.setup(&nh, pcd_file);

    // Visualization
    pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");

    viewer_->runOnVisualizationThread(viz_cb, "viz_cb");
    
    ros::spin();
}
