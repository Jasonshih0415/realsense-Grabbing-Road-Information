#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class DepthImageProcessor:
    def __init__(self):
        # Initialize the ROS node and the CvBridge
        rospy.init_node('depth_image_processor')
        self.bridge = CvBridge()
        self.depth_pub = rospy.Publisher('/camera/depth_zero', Image, queue_size=10)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.process_and_publish_depth_image)
        self.depth_scale = rospy.get_param('/camera/depth_scale',0.001)
 
    def process_and_publish_depth_image(self, raw_depth_image):
        # Convert the ROS Image message to a NumPy array (OpenCV format)
        cv_image = self.bridge.imgmsg_to_cv2(raw_depth_image, desired_encoding='passthrough')

        # Retain columns from 160 to 480, set other columns to zero
        processed_image = np.zeros_like(cv_image)  # Create a zero array with the same shape as cv_image
        processed_image[:, 160:480] = cv_image[:, 160:480]  # Retain data from columns 160 to 480
        processed_image[processed_image > 3000.0] = 0
        
        min_value = np.min(processed_image)
        max_value = np.max(processed_image)
        
        # Print min and max values
        #rospy.loginfo(f'Min value in processed image: {min_value}')
        #rospy.loginfo(f'Max value in processed image: {max_value}')
        
        
        # Convert the processed NumPy array back to a ROS Image message
        processed_depth_image = self.bridge.cv2_to_imgmsg(processed_image, encoding='passthrough')
        
        # Set the timestamp of the processed image to match the original image
        processed_depth_image.header.stamp = raw_depth_image.header.stamp
        processed_depth_image.header.frame_id = raw_depth_image.header.frame_id  # Optional: keep the frame ID consistent

        # Publish the processed depth image to the 'depth_zero' topic
        self.depth_pub.publish(processed_depth_image)

# Start the processing
if __name__ == '__main__':
    try:
        DepthImageProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


