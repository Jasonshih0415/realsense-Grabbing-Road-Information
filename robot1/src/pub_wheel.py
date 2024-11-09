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
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Initialize the ROS node
        rospy.init_node('pointcloud_filter_node', anonymous=True)

        # Subscribe to the point cloud topic
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.callback_pointcloud)
        # Set up publishers for right and left filtered points
        self.publisher = rospy.Publisher('/filtered_points', PointCloud2, queue_size=10)
        
    def callback_pointcloud(self, data):
        start_time = time.time()

        # Ensure the message is of type PointCloud2
        assert isinstance(data, PointCloud2)

        # Convert PointCloud2 to a numpy array with (x, y, z) points
        points = np.fromiter(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True), 
                             dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        #change data to (N,3)
        self.points = np.round(np.vstack((points['x'], points['y'], points['z'])).T, 2)
        
        processing_time = time.time() - start_time
       
        self.points = self.points[(self.points[:, 0] == 0.14) | (self.points[:, 0] == -0.14)]
            
        print("Processing time: {:.6f} sec".format(processing_time))

        # Publish the filtered points
        self.publish_filtered_points()

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
        if len(self.points) > 0:
            cloud_msg = point_cloud2.create_cloud(header, fields, self.points)
            self.publisher.publish(cloud_msg)


if __name__ == '__main__':
    # Instantiate the visualizer and keep the node running
    visualizer = PointCloudVisualizer()
    rospy.spin()

