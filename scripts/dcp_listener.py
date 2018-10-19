#!/usr/bin/env python
# -*- coding: utf-8 -*-

#ROS libraries
import roslib
roslib.load_manifest('dcp_dehaze')
import sys
import rospy
import rospkg
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#Python libraries
import numpy as np
import cv2
from yaml import load, dump, Loader, Dumper
#Classes
from dcp_dehaze import DCPDehaze

# ===CALLBACK TO PERFORM DEHAZING
def dcp_callback(stereo_image_msg, depth_msg):

    #print("Got it")
    # Global params - TODO read them from a config file
    global radiance_pub
    global dehazer

    # Receive the images and transform them into arrays
    my_cvbridge = CvBridge()
    stereo_image = my_cvbridge.imgmsg_to_cv2(stereo_image_msg,"bgr8")
    depth_image = my_cvbridge.imgmsg_to_cv2(depth_msg,"32FC1")

    # Dehaze image
    radiance = dehazer.dehaze(stereo_image,depth_image)
    #print(dehazer.enable_underwater)

    # Convert to ROS msg and publish
    radiance_msg = my_cvbridge.cv2_to_imgmsg(radiance, "bgr8")
    radiance_msg.header = stereo_image_msg.header
    radiance_pub.publish(radiance_msg)

# ===PUBLISHERS AND SUBSCRIBERS
def init():

    global radiance_pub
    global dehazer

    config_file = rospy.get_param('~config_filename')
    config_dict = load(file(config_file, 'r'), Loader=Loader)

    dehazer = DCPDehaze(trans_min=config_dict['minimum_transmission'],
                        atm_max=config_dict['max_atm_light'],
                        window_size=config_dict['dehaze_window_size'],
                        guided_filter_radius=config_dict['guided_filter_radius'],
                        enable_underwater=config_dict['enable_underwater'],
                        attenuation_coeffs=config_dict['light_attenuation_coeffs'],
                        enable_wb=config_dict['enable_white_balance'])

    radiance_pub = rospy.Publisher(config_dict['dehazed_image_topic'], Image, queue_size=10)
    image_sub = message_filters.Subscriber(config_dict['hazed_image_topic'], Image)
    depth_sub = message_filters.Subscriber(config_dict['depth_image_topic'], Image)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.1)
    ts.registerCallback(dcp_callback)


# ===MAIN
if __name__ == '__main__':
    try:
        rospy.init_node('dcp_listener')
        init()

    except rospy.ROSInterruptException:
        pass

    rospy.spin()
