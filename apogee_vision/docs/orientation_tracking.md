# Tracking

The shape of the object can be simplified so we can track a bounding box rather than the 3D point cloud which is much more computationally expensive.

You can also average the differentiated orientation quaternion to calculate the angular velocity rather than using a kalman filter.

Subtracting two quaternions in the right way results in a 3D vector.