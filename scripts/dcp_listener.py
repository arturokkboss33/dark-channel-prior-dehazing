#!/usr/bin/env python
# -*- coding: utf-8 -*-

#ROS libraries
import roslib
roslib.load_manifest('dcp_uw_dehaze')
import sys
import rospy
import rospkg
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#Python libraries
import numpy as np
import cv2
#Classes
from dcp_dehaze import DCPDehaze



def white_balance(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result

# ===CALLBACK TO PERFORM DEHAZING
def dcp_callback(stereo_image_msg, depth_msg):

    print("Got it")
    # Global params - TODO read them from a config file
    global radiance_pub
    global dehazer

    # Receive the images and transform them into arrays
    my_cvbridge = CvBridge()
    stereo_image = my_cvbridge.imgmsg_to_cv2(stereo_image_msg,"bgr8")
    depth_image = my_cvbridge.imgmsg_to_cv2(depth_msg,"32FC1")

    # Optional - Insert white balance method for further refinement
    wb_stereo_image = white_balance(stereo_image)

    # Dehaze image
    radiance = dehazer.dehaze(stereo_image,depth_image)
    print(dehazer.enable_underwater)

    # Convert to ROS msg and publish
    radiance_msg = my_cvbridge.cv2_to_imgmsg(radiance, "bgr8")
    radiance_msg.header = stereo_image_msg.header
    radiance_pub.publish(radiance_msg)

# ===PUBLISHERS AND SUBSCRIBERS
def init():

    global radiance_pub
    global dehazer

    dehazer = DCPDehaze(trans_min=0.2,atm_max=220,window_size=15,guided_filter_radius=40,
                        enable_underwater=True,attenuation_coeffs=[0.005,0.01,0.01])
    radiance_pub = rospy.Publisher('/stereo_camera/left/radiance', Image, queue_size=10)
    image_sub = message_filters.Subscriber(
        '/stereo_camera/left/image_uncomp', Image)
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
