#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('dcp_uw_dehaze')
import sys
import rospy
import rospkg
import message_filters
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
#from functools import partial
#from PIL import Image as PILImage
#from dehaze import dehaze

# ===CALLBACK TO PERFORM DEHAZING


def dcp_callback(stereo_image_msg, depth_msg):

    # Global params - TODO read them from a config file
    global radiance_pub
    attenuation_coeff = 0.5
    atmospheric_vals = np.zeros((1,1,3),dtype=np.float)
    atmospheric_vals[0,0,0] = 184
    atmospheric_vals[0,0,1] = 128
    atmospheric_vals[0,0,2] = 9

    # Receive the images and transform them into arrays
    my_cvbridge = CvBridge()
    stereo_image = my_cvbridge.imgmsg_to_cv2(stereo_image_msg,"bgr8");
    depth_image = my_cvbridge.imgmsg_to_cv2(depth_msg,"32FC1");

    # Convert the depth image into grayscale and compute transmission function    
    depth_image = np.nan_to_num(depth_image)*255.
    depth_image = np.maximum(np.minimum(depth_image, 255.), 0.0001)
    transmission = np.exp(-attenuation_coeff*depth_image)
    tiled_t = np.zeros(stereo_image.shape,dtype=np.float32)
    tiled_t[:, :, 0] = tiled_t[:, :, 1] = tiled_t[:, :, 2] = transmission

    # Compute radiance R = (I-A)/t+A
    radiance_raw = (stereo_image.astype(float) - atmospheric_vals) / tiled_t + atmospheric_vals
    radiance = np.maximum(np.minimum(radiance_raw, 255), 0).astype(np.uint8)

    # Convert to ROS msg and publish
    radiance_msg = my_cvbridge.cv2_to_imgmsg(radiance, "bgr8")
    radiance_msg.header = stereo_image_msg.header
    radiance_pub.publish(radiance_msg)




# ===PUBLISHERS AND SUBSCRIBERS
def init():

    global radiance_pub
    radiance_pub = rospy.Publisher('/stereo_camera/right/radiance', Image, queue_size=10)

    image_sub = message_filters.Subscriber(
        '/stereo_camera/right/image_uncomp', Image)
    depth_sub = message_filters.Subscriber('/depth_camera/depth/image_raw', Image)

    ts = message_filters.ApproximateTimeSynchronizer(
        [image_sub, depth_sub], 10, 0.1)
    ts.registerCallback(dcp_callback)


# ===MAIN
if __name__ == '__main__':
    try:
        rospy.init_node('dcp_listener')
        init()

    except rospy.ROSInterruptException:
        pass

    rospy.spin()
