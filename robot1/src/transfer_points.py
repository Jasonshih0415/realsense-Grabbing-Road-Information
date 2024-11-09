#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np
import std_msgs.msg
import time
from geometry_msgs.msg import Vector3

class PointCloudVisualizer:
    def __init__(self):
        # Initialize arrays to store points
        self.points = np.empty((0, 3))
        self.transformed_points = np.empty((0, 3))
        self.path_points = np.empty((0, 3))
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Initialize the ROS node
        rospy.init_node('pointcloud_transform_node', anonymous=True)

        # Subscribe to the point cloud topic
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.callback_pointcloud)
        rospy.Subscriber('/imu/rpy', Vector3, self.callback_imu)

        # Set up a publisher for the transformed and filtered points
        self.publisher = rospy.Publisher('/filtered_points_transform', PointCloud2, queue_size=10)

    def callback_pointcloud(self, data):
        start_time = time.time()
        # Ensure the message is of type PointCloud2
        #assert isinstance(data, PointCloud2)
        
        # Convert PointCloud2 to a numpy array with (x, y, z) points
        points = np.fromiter(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True), 
                             dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        #print("shape:",points.shape)
        read_time_1 = time.time()
        #change data to (N,3)
        self.points = np.round(np.vstack((points['x'], points['y'], points['z'])).T, 2)
        
        read_time_2 = time.time()
	
        # Apply rotation and translation transformation if there are points
        if len(self.points) > 0:
            self.transformed_points = self.apply_rpy_transformation(self.points)
        self.path_points=np.round(self.transformed_points,2) 
        
        transform_time = time.time()
        
        self.path_points = self.path_points[(self.path_points[:, 0] == 0.14) | (self.path_points[:, 0] == -0.14)]# all row and first column, get x of every points
        #print("transformed_points:",self.transformed_points.shape)
        #self.path_points_y= self.path_points[(self.path_points[:, 1] > self.height)]               
        #print("num_path_point:",self.path_points.shape)
        
        capture_time = time.time()
        #print("read_time_1: {:.6f} sec".format(read_time_1 - start_time))
        #print("read_time_2: {:.6f} sec".format(read_time_2 - read_time_1))
        #print("transform_time: {:.6f} sec".format(transform_time - read_time_2))
        #print("capture_time: {:.6f} sec".format(capture_time - transform_time))
        #print("====")

        # Publish the filtered points
        self.publish_filtered_points()

    def apply_rpy_transformation(self, points):
        # Calculate the angles for rotation
        self.alpha = np.radians(self.pitch + 90)
        self.phi = np.radians(self.roll)

        # Translation parameters
        self.height = 0.36  # Camera height above ground
        self.angle = np.abs(-90 - self.pitch)
        self.px = 0
        self.py = -self.height * np.sin(np.radians(90 - self.angle))
        
        if self.alpha < 0:
            self.pz = self.height * np.sin(np.radians(self.angle))
        else:
            self.pz = -self.height * np.sin(np.radians(self.angle))
        
        translation = np.array([self.px, self.py, self.pz])
        points += translation

        # Rotation matrices for pitch and roll
        pitch_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(self.alpha), -np.sin(self.alpha)],
            [0, np.sin(self.alpha), np.cos(self.alpha)]
        ])

        roll_matrix = np.array([
            [np.cos(self.phi), -np.sin(self.phi), 0],
            [np.sin(self.phi), np.cos(self.phi), 0],
            [0, 0, 1]
        ])

        # Combine the rotation matrices into a single transformation matrix
        rotation_matrix = roll_matrix @ pitch_matrix

        # Apply the transformation to each point in the point cloud
        transformed_points = points @ rotation_matrix.T  # Matrix multiplication

        return transformed_points

    def callback_imu(self, data):
        # Update roll, pitch, and yaw from the IMU data
        self.pitch = data.x
        self.roll = data.y
        self.yaw = data.z

    def publish_filtered_points(self):
        # Create header for the PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_color_optical_frame"  # Adjust frame ID if necessary

        # Define the point fields for the PointCloud2 message
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        # Only publish if path_points has data
        if len(self.path_points) > 0:
            cloud_msg = point_cloud2.create_cloud(header, fields, self.path_points)
            self.publisher.publish(cloud_msg)

if __name__ == '__main__':
    # Instantiate the visualizer and keep the node running
    visualizer = PointCloudVisualizer()
    rospy.spin()

