import pyrealsense2 as rs
import numpy as np
import cv2
from PIL import Image


def configureCamera():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    return pipeline, config

def captureImageAndDepth(pipeline, config): 
    pipeline.start(config)
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    color_colormap_dim = color_image.shape
    # print(color_colormap_dim)
    pipeline.stop()
    return color_image, depth_frame

def segmentImage(npImageArray):
    hsv_image = cv2.cvtColor(npImageArray, cv2.COLOR_BGR2HSV)

    #thresholds for HSV filter
    channel1_min = 0.000 * 179
    channel1_max = 0.046 * 179
    channel2_min = 0.586 * 255
    channel2_max = 1.000 * 255
    channel3_min = 0.196 * 255
    channel3_max = 0.945 * 255

    # mask based on the defined thresholds
    mask = cv2.inRange(hsv_image, 
                    (channel1_min, channel2_min, channel3_min), 
                    (channel1_max, channel2_max, channel3_max))
    
    moments = cv2.moments(mask)
    # Calculate ball Centroid
    centroid_x = int(moments['m10'] / moments['m00'])
    centroid_y = int(moments['m01'] / moments['m00'])
    return mask, centroid_x, centroid_y

def displayImageWFilter(mask, centroid_x, centroid_y, colorImage):
    result = cv2.bitwise_and(colorImage, colorImage, mask=mask)
    cv2.imshow('Result', result)
    cv2.waitKey(0)
    centroid_color = (0, 255, 0) 
    radius = 5 

    result = cv2.bitwise_and(colorImage, colorImage, mask)
    cv2.circle(result, (centroid_x, centroid_y), radius, centroid_color, -1)
    # Display the result
    cv2.imshow('Result', result)
    cv2.waitKey(0)

def returnVectorInfo(centroid_x, centroid_y, depth):
    depth_intrin = depth.profile.as_video_stream_profile().intrinsics

    depth = depth.get_distance(centroid_x, centroid_y)
    depth_point = rs.rs2_deproject_pixel_to_point(
    depth_intrin, [centroid_x, centroid_y], depth)
    
    return depth_point


def main():
    pipeline, config = configureCamera()
    colorImage, depth = captureImageAndDepth(pipeline, config)
    #cv2.imwrite('correctBall.jpeg', colorImage)
    mask, centroid_x, centroid_y = segmentImage(colorImage)
    # Uncomment Display Image if want to disp img
    #displayImageWFilter(mask, centroid_x, centroid_y, colorImage)

    depth_point = returnVectorInfo(centroid_x, centroid_y, depth)
    dist = depth.get_distance(centroid_x, centroid_y)
    # print(dist)
            
    print("x: " + str(depth_point[0]) + " y: " + str(depth_point[1]) + " z: " + str(depth_point[2]))

main()

