#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np
import std_msgs.msg
import time
from geometry_msgs.msg import Vector3
import ros_numpy

class PointCloudVisualizer:
    def __init__(self):
        # Initialize arrays to store points
        self.points = np.empty((0, 3))
        self.transformed_points = np.empty((0, 3))
        self.path_points = np.empty((0, 3))
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.height = rospy.get_param('/camera/pointcloud_transform_node/height', 0.395) 
        #print(f"Height parameter set to: {self.height}===================================================================")

        # Initialize the ROS node
        rospy.init_node('pointcloud_transform_node', anonymous=True)

        # Subscribe to the point cloud topic
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.callback_pointcloud)
        rospy.Subscriber('/imu/rpy', Vector3, self.callback_imu)

        # Set up a publisher for the transformed and filtered points
        self.publisher = rospy.Publisher('/filtered_points_transform', PointCloud2, queue_size=10)

    def callback_pointcloud(self, data):
        start_time = time.time()
        pc = ros_numpy.numpify(data)
        x = pc['x']
        y = pc['y']
        z = pc['z']
        valid_mask = ~np.isnan(x) & ~np.isnan(y) & ~np.isnan(z)
        x_valid = np.round(x[valid_mask],2)
        y_valid = np.round(y[valid_mask],3)
        z_valid = np.round(z[valid_mask],3)
        self.points = np.stack((x_valid, y_valid, z_valid), axis=-1)
        print("shape:",self.points.shape)
        read_time_1 = time.time()

        
        self.o_path= self.points[(self.points[:, 0] == 0.14) | (self.points[:, 0] == -0.14)]
        if len(self.o_path) > 0:
            self.transformed_points = self.apply_rpy_transformation(self.o_path)
        self.path_points = np.copy(self.transformed_points)  # Create a copy of the original array
        self.path_points[:, 0] = np.round(self.transformed_points[:, 0], 2)  # Round x (first column) to 2 decimal places
        self.path_points[:, 1] = np.round(self.transformed_points[:, 1], 3)  # Round y (second column) to 3 decimal places
        self.path_points[:, 2] = np.round(self.transformed_points[:, 2], 3)
        
        print("read_time",read_time_1-start_time)
        self.publish_filtered_points(self.path_points)

    def apply_rpy_transformation(self, points):
        # Calculate the angles for rotation
        self.alpha = np.radians(self.pitch)
        self.phi = np.radians(self.roll)

        # Translation parameters
        self.angle = np.abs(0 - self.pitch)
        self.px = 0
        self.py = -self.height * np.sin(np.radians(90 - self.angle))
        
        if self.alpha < 0:
            self.pz = -self.height * np.sin(np.radians(self.angle))
        else:
            self.pz = self.height * np.sin(np.radians(self.angle))
        
        #print(f'self.py:{self.py},self.pz:{self.pz}')
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
        #print(f"pitch:{self.pitch},yaw:{self.yaw},roll:{self.roll}")
    def publish_filtered_points(self,point):
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
        if len(point) > 0:
            cloud_msg = point_cloud2.create_cloud(header, fields, point)
            self.publisher.publish(cloud_msg)

if __name__ == '__main__':
    # Instantiate the visualizer and keep the node running
    visualizer = PointCloudVisualizer()
    rospy.spin()

