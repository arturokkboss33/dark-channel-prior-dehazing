#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('dcp_uw_dehaze')
import sys
import rospy
import rospkg
import message_filters
from sensor_msgs.msg import Image
from functools import partial
from PIL import Image as PILImage
from dehaze import dehaze

# ===CALLBACK TO PERFORM DEHAZING


def dcp_callback(depth,stereo_image):
    print("Got an image")

# ===PUBLISHERS AND SUBSCRIBERS


def init():

    depth_sub = message_filters.Subscriber('/depth_camera/depth/image_raw', Image)
    image_sub = message_filters.Subscriber(
        '/stereo_camera/right/image_uncomp', Image)

    ts = message_filters.ApproximateTimeSynchronizer(
        [depth_sub, image_sub], 10, 0.1)
    ts.registerCallback(dcp_callback)


# ===MAIN
if __name__ == '__main__':
    try:
        rospy.init_node('dcp_listener')
        init()

    except rospy.ROSInterruptException:
        pass

    rospy.spin()
