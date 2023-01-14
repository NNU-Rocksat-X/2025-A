
#include <tracker_lib.h>


/////////////////////////////////////////////////////
//                Static Functions
////////////////////////////////////////////////////

static void Xrotate(Eigen::Matrix3f &matrix, float angle)
{
    Eigen::Matrix3f mat = Eigen::Matrix3f::Identity();
    mat(1, 1) = cos(angle);
    mat(1, 2) = -1 * sin(angle);
    mat(2, 1) = sin(angle);
    mat(2, 2) = cos(angle);
    matrix = matrix * mat;
}

static void rotMatrixToQuaternion(Eigen::Matrix3f &mat, geometry_msgs::Quaternion &quaternion)
{
    quaternion.w = sqrt(1 + mat(0,0) + mat(1, 1) + mat(2, 2)) / 2; // W
    quaternion.x = (mat(2,1) - mat(1,2)) / (4 * quaternion.w);  // x
    quaternion.y = (mat(0,2) - mat(2,0)) / (4 * quaternion.w);  // y
    quaternion.z = (mat(1,0) - mat(0,1)) / (4 * quaternion.w);  // z
}

static void filterPassThrough(const CloudConstPtr &cloud, Cloud &result)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 10.0);
    pass.setKeepOrganized(false);
    pass.setInputCloud(cloud);
    pass.filter(result);
}

static void gridSampleApprox(const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
    pcl::ApproximateVoxelGrid<RefPointType> grid;
    grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float>(leaf_size));
    grid.setInputCloud(cloud);
    grid.filter(result);
}

/////////////////////////////////////////////////////
//                TrackerLib
////////////////////////////////////////////////////

TrackerLib::TrackerLib() 
{
    tracker_ = KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>(8);
}

void TrackerLib::setup(ros::NodeHandle* nh, char* pcd_file)
{
    pose_pub = nh->advertise<geometry_msgs::Pose>("/wrench_pose", 10);
    point_cloud_sub = nh->subscribe("/point_cloud", 1, &TrackerLib::cloud_cb, this);

    // Read PCD File
    target_cloud.reset(new Cloud());
    if (pcl::io::loadPCDFile(pcd_file, *target_cloud) == -1)
    {
        ROS_ERROR("Reference PCD file not found.");
        assert(0);
    }

    // Initial guess
    initial_transformation_ = Eigen::Affine3f::Identity();
    initial_transformation_.linear() = (Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
                                * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
                                * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())).toRotationMatrix();
    initial_transformation_.scale(Eigen::Vector3f(0.007, 0.007, 0.007));
    initial_transformation_.translation() = Eigen::Vector3f(1.5, 1, 1); 
    pcl::transformPointCloud<RefPointType> (*target_cloud, *target_cloud, initial_transformation_);

    // Set Parameters
    std::vector<double> default_step_covariance;
    std::vector<double> initial_noise_covariance;
    std::vector<double> default_initial_mean;
    int maxParticleNum;
    int particleNum;
    float delta;
    float resampleLikelihood;
    float epsilon;
    if (ros::param::has("/ParticleFilterConfig/step_covariance"))
    {
        ros::param::get("/ParticleFilterConfig/step_covariance", default_step_covariance);
        ros::param::get("/ParticleFilterConfig/initial_noise_covariance", initial_noise_covariance);
        ros::param::get("/ParticleFilterConfig/default_initial_mean", default_initial_mean);
        ros::param::get("/ParticleFilterConfig/downsampling_grid_size", downsampling_grid_size_);
        ros::param::get("/ParticleFilterConfig/maxParticleNum", maxParticleNum);
        ros::param::get("/ParticleFilterConfig/particleNum", particleNum);
        ros::param::get("/ParticleFilterConfig/delta", delta);
        ros::param::get("/ParticleFilterConfig/resampleLikelihood", resampleLikelihood);
        ros::param::get("/ParticleFilterConfig/epsilon", epsilon);
    } else {
        ROS_ERROR("Load Particle Filter Config to parameter server!");
    }

    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;

    //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    tracker_.setMaximumParticleNum(maxParticleNum);
    tracker_.setDelta(delta);
    tracker_.setEpsilon(epsilon);
    tracker_.setBinSize(bin_size);

    //Set all parameters for  ParticleFilter
    tracker_.setTrans(Eigen::Affine3f::Identity());
    tracker_.setStepNoiseCovariance(default_step_covariance);
    tracker_.setInitialNoiseCovariance(initial_noise_covariance);
    tracker_.setInitialNoiseMean(default_initial_mean);
    tracker_.setIterationNum(1);
    tracker_.setParticleNum(particleNum);
    tracker_.setResampleLikelihoodThr(resampleLikelihood);
    tracker_.setUseNormal(false);

    // Setup coherence object for tracking
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence
        (new ApproxNearestPairPointCloudCoherence<RefPointType>);

    DistanceCoherence<RefPointType>::Ptr distance_coherence
        (new DistanceCoherence<RefPointType>);

    pcl::search::Octree<RefPointType>::Ptr search(new pcl::search::Octree<RefPointType> (0.01));
    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(0.01);

    tracker_.setCloudCoherence(coherence);

    Eigen::Vector4f centroid;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    CloudPtr transed_ref(new Cloud);
    CloudPtr transed_ref_downsampled(new Cloud);

    pcl::compute3DCentroid<RefPointType> (*target_cloud, centroid);
    trans.translation().matrix() = Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
    pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
    gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

    tracker_.setReferenceCloud(transed_ref_downsampled);
    tracker_.setTrans(trans);
}

void TrackerLib::publishResult(void)
{
    ParticleXYZRPY result = tracker_.getResult();
    Eigen::Affine3f transformation = tracker_.toEigenMatrix(result);
    Eigen::Vector3f translation = transformation.translation();
    Eigen::Matrix3f rotation = transformation.rotation();

    //ROS_INFO("Translation: %f, %f, %f", translation(0), translation(1), translation(2));

    object_pose_.position.x = translation(0);
    object_pose_.position.y = translation(1);
    object_pose_.position.z = translation(2);

    rotMatrixToQuaternion(rotation, object_pose_.orientation);

    pose_pub.publish(object_pose_); 
}

void TrackerLib::cloud_cb (const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud_msg)
{
    // Convert from ros PointCloud2 to PCL point cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    
    // Filter
    cloud_pass_.reset(new Cloud);
    cloud_pass_downsampled_.reset(new Cloud);
    cloud_pass_ = cloud;
    // Filters some points out, but usually ends up slicing in undesired way
    //cloud_pass_downsampled_ = cloud;
    //filterPassThrough(cloud, *cloud_pass_);
    gridSampleApprox(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

    if (counter < 50)
    {
        counter++;
    } else {
        // Track object
        tracker_.setInputCloud(cloud_pass_downsampled_);
        tracker_.compute();
    }

    new_cloud_ = true;
}

void TrackerLib::calculate(void)
{
    if (new_cloud_ && cloud_pass_downsampled_)
    {      
        ParticleFilter::PointCloudStatePtr particles = tracker_.getParticles();
        if (particles)
        {
            publishResult();
        }
    }
    new_cloud_ = false;
    
}

CloudConstPtr TrackerLib::get_reference(void)
{
    return tracker_.getReferenceCloud();
}

uint8_t TrackerLib::get_recv_cloud(CloudPtr cloud)
{
    if (!cloud_pass_)
    {
        return false;
    }
    else {
        copyPointCloud(*cloud_pass_, *cloud);
        return true;
    }
}

void TrackerLib::get_transformation(Eigen::Affine3f &transformation)
{
    ParticleXYZRPY result = tracker_.getResult();
    transformation = tracker_.toEigenMatrix(result);
}

ParticleFilter::PointCloudStatePtr TrackerLib::get_particles()
{
    return tracker_.getParticles();
}

TrackerLib::~TrackerLib() {}
