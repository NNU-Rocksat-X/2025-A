#!/usr/bin/env python3

import numpy as np
import math

import open3d as o3d


class DepthCam():

    # shape is shape of frame (num_x_pix, num_y_pix)
    # fovy is field of view angle in degrees
    def __init__(self, fovy: float, width: int, height: int) -> (float, float, float): # may need ipd (inter-pupilary distance)
        self.fovy = fovy # AFOV
        self.fovx = fovy * (width/height)

        self.pxToMeter = 1
        self.focalx = (width/2) / math.tan(self.fovx * math.pi/180)
        self.focaly = (height/2) / math.tan(self.fovy * math.pi/180)

        self.cam_intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, self.focalx, self.focaly, 500, 0)
        

    def pix_to_point(self, x_pix, y_pix, d):
        d *= self.pxToMeter
        x_over_z = x_pix / self.focalx
        y_over_z = y_pix / self.focaly
        z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
        x = x_over_z * x_over_z
        y = y_over_z * x_over_z
        return x, y, z

    def intrinsics(self):
        return self.cam_intrinsics
        