#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo

def image_callback(msg):
    # Print the timestamp of the image message
    rospy.loginfo("Image Timestamp: %s" % msg.header.stamp.to_sec())

def camera_info_callback(msg):
    # Print the timestamp of the camera info message
    rospy.loginfo("CameraInfo Timestamp: %s" % msg.header.stamp.to_sec())

def listener():
    rospy.init_node('timestamp_listener', anonymous=True)

    rospy.Subscriber('/camera/depth_zero', Image, image_callback)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

