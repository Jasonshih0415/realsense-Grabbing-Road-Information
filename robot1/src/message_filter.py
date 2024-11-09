#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
import message_filters

# Publisher for the synchronized depth image and camera info topics
synchronized_depth_pub = None
synchronized_camera_info_pub = None

def callback(depth_msg, camera_info_msg):
    rospy.loginfo("Received depth image and camera info")
    rospy.loginfo("Depth timestamp: %s, Camera info timestamp: %s", depth_msg.header.stamp, camera_info_msg.header.stamp)
    
    # Publish both depth image and camera info
    synchronized_depth_pub.publish(depth_msg)
    synchronized_camera_info_pub.publish(camera_info_msg)

def main():
    global synchronized_depth_pub, synchronized_camera_info_pub

    rospy.init_node('message_filter_node')

    # Publisher for the synchronized depth image and camera info topics
    synchronized_depth_pub = rospy.Publisher('/camera/synchronized_depth', Image, queue_size=10)
    synchronized_camera_info_pub = rospy.Publisher('/camera/synchronized_camera_info', CameraInfo, queue_size=10)

    # Subscribe to the depth image and camera info topics
    depth_sub = message_filters.Subscriber('/camera/depth_zero', Image)
    camera_info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)

    # Use ApproximateTimeSynchronizer to synchronize the messages
    ts = message_filters.ApproximateTimeSynchronizer([depth_sub, camera_info_sub], queue_size=10, slop=0.5)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()

