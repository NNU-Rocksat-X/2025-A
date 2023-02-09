# PCD Generation
A PCD (Point Cloud Data) file is used in the object detection and tracking algorithm. 
The depth camera produces a 3D point cloud which is compare to the PCD for that object.

A PCD can be generate using the pcl_mesh2pcd program. 
(Included in pcl-1.8.1 which is required to run to tracker)

 pcl_mesh2pcd takes a .ply as input, howerever an stl can be converted to a ply [here](https://imagetostl.com/convert/file/stl/to/ply)
