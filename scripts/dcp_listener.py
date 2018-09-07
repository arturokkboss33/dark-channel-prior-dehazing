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


def white_balance(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result


def get_atmosphere(img, depth):
    
    M, N = depth.shape
    flatdepth = depth.ravel()
    flatdepth = np.nan_to_num(flatdepth)*255.
    flatdepth = np.maximum(np.minimum(flatdepth, 255.), 0.0001)/255.
    #searchidx = np.argwhere(np.isnan(flatdepth))
    flatimg = img.reshape(M*N,3)
    atm_vals = np.average(flatimg, axis=0, weights=1./flatdepth)
    print(atm_vals)
    return atm_vals
    # reference CVPR09, 4.4
    # M, N = darkch.shape
    # flatI = I.reshape(M * N, 3)
    # flatdark = darkch.ravel()
    # searchidx = (-flatdark).argsort()[:M * N * p]  # find top M * N * p indexes
    # print 'atmosphere light region:', [(i / N, i % N) for i in searchidx]

    # # return the highest intensity for each channel
    # return np.max(flatI.take(searchidx, axis=0), axis=0)


# ===CALLBACK TO PERFORM DEHAZING
def dcp_callback(stereo_image_msg, depth_msg):

    print("Got it")
    # Global params - TODO read them from a config file
    global radiance_pub
    attenuation_coeff = np.array([0.005,0.01,0.01])
    # atmospheric_vals = np.zeros((1,1,3),dtype=np.float)
    # atmospheric_vals[0,0,0] = 113
    # atmospheric_vals[0,0,1] = 154
    # atmospheric_vals[0,0,2] = 47

    # Receive the images and transform them into arrays
    my_cvbridge = CvBridge()
    stereo_image = my_cvbridge.imgmsg_to_cv2(stereo_image_msg,"bgr8")
    depth_image = my_cvbridge.imgmsg_to_cv2(depth_msg,"32FC1")

    # Optional - Insert white balance method for further refinement
    wb_stereo_image = white_balance(stereo_image)

    # Compute the atmospheric light
    atmospheric_vals = np.minimum(get_atmosphere(stereo_image,depth_image),255)
    #atmospheric_vals = np.zeros((3),dtype=np.float)
    atmospheric_vals[0] = 255.
    atmospheric_vals[1] = 180.
    atmospheric_vals[2] = 18.
    #print(atmospheric_vals)

    # Convert the depth image into grayscale and compute transmission function    
    depth_image = np.nan_to_num(depth_image)*255.
    depth_image = np.maximum(np.minimum(depth_image, 255.), 0.0001)
    transmission_B = np.exp(-attenuation_coeff[0]*depth_image)
    transmission_G = np.exp(-attenuation_coeff[1]*depth_image)
    transmission_R = np.exp(-attenuation_coeff[2]*depth_image)
    tiled_t = np.zeros(stereo_image.shape,dtype=np.float32)
    tiled_t[:, :, 0] = transmission_B
    tiled_t[:, :, 1] = transmission_G
    tiled_t[:, :, 2] = transmission_R

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
