# Apogee Vision

Apogee Vision contains files to create and publish 3d point clouds, and track objects.

## Organization
./apogee_vision/

    Launch Files
    ----------------------------------------
    launch/
        track_object.launch -- Launches processes to publish point cloud and track object in that point cloud

    Executables
    ------------------------------------------
    src/
        object_tracking.cpp -- Creates ros node for object tracking library
        object_tracking_gui.cpp -- Shows gui for object tracking library (Debugging)
        point_cloud.cpp -- Creates and publishes the point cloud from the depth camera
        rviz_tracked_object.cpp -- Creates an object in rviz to view the pose of the tracked object

    scripts/
        apply_force -- Applies force to simulated object for testing
        cam_frame -- Publishes simulate camera's position
        depth_view -- Creates GUI window to view depth camera depth frame
        icp -- Testing use of ICP for object detection (Didnt work very well)
        view -- Creates GUI window to view depth camera's RGB frame

    Libraries
    --------------------------------------------
    src/
        tracker_lib.cpp -- Tracks object with PCL particle filter
        
## Installing

### Install PCL (Point Cloud Library)
`wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.8.1.zip`

`unzip pcl-1.8.1.zip`

`cd pcl-pcl-1.8.1`

`mkdir build`

`cd build`

`cmake ..`

`make`

