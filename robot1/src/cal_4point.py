#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np
import time
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import quaternion_from_euler
import math
import serial
import std_msgs.msg
from scipy.ndimage import gaussian_filter1d
import ros_numpy

class PointCloudVisualizer:
    def __init__(self):
        self.points_x_minus_0_14 = np.empty((0, 3))  # Initialize with an empty array
        self.points_x_0_14 = np.empty((0, 3))        # Initialize the other array
        self.serial_port = '/dev/ttyUSB0'  # Change this to your ESP32's port
        self.baud_rate = 115200  # Change this to the baud rate set on the ESP32
        self.height = rospy.get_param('/camera/pointcloud_transform_node/height', 0.395)
        rospy.init_node('cal_4points', anonymous=True)
        self.pitch_x = 0
        self.y_change = 0
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            self.serial_port = '/dev/ttyUSB1'
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
                rospy.loginfo(f"Successfully opened fallback serial port: {self.serial_port}")
            except serial.SerialException as e:
                rospy.logerr(f"Failed to open fallback serial port: {e}")
                raise e  # Re-raise if the fallback also fails
                
        rospy.Subscriber('/filtered_points_transform', PointCloud2, self.callback_pointcloud)
        self.pose_array_pub = rospy.Publisher("pose_array_topic", PoseArray, queue_size=10)
        self.publisher = rospy.Publisher('filter_point', PointCloud2, queue_size=10)

    def average_y(self, points, height_threshold=0.1):
        if points.size == 0:
            rospy.logwarn("Input points array is empty.")
            return np.array([])
        # Find unique z values and sort points by z
        unique_z, indices = np.unique(points[:, 2], return_inverse=True)
    
        # Compute mean y for each unique z value
        mean_y = np.bincount(indices, weights=points[:, 1]) / np.bincount(indices)

        # Construct the averaged data
        x = points[0, 0]
        averaged_data = np.column_stack((np.full_like(unique_z, x, dtype=float), mean_y, unique_z))
        return np.round(averaged_data, 3)
        
    def calculate_pitch(self,f_point,b_point):
        dy = b_point[1]-f_point[1]
        dz = -(b_point[2]-f_point[2])
        pitch = np.arctan2(dy, dz) #use negative because the dATA is upside down to the coordinate, see rviz know the sign
        pitch_deg = np.round(np.degrees(pitch),1)
        '''
        print("pitch_deg",pitch_deg)
        print("f_point[1]",f_point[1])
        print("b_point[1]",b_point[1])
        print("f_point[2]",f_point[2])
        print("b_point[2]",b_point[2])
        print("dy ",dy)
        print("dz",dz)
        print("----------")
        '''
        return pitch_deg,dy,dz
      
    def average_filter(self,point_array, kernel_size=7):#average y base on z axis
        # initial
        input_array = point_array[:,1]
        kernel = np.ones(kernel_size) 
        input_len = len(input_array)
        kernel_len = len(kernel)
        kernel_middle = kernel_len // 2
        
        # convolution setting
        padding = kernel_middle
        padded_input = np.pad(input_array, (kernel_middle, kernel_middle), mode='constant', constant_values=0)
        output = [0] * input_len

        # convolution operation
        for i in range(input_len):
            conv_sum = 0
            zero_num = 0
            for j in range(kernel_len):
                if padded_input[i + j]==0:
                    zero_num += 1
                conv_sum += kernel[j] * padded_input[i + j]
            # Assign the result to the middle of the kernel's range
            denominator = kernel_len - zero_num
            if denominator == 0:
                output[i] = 0  # Or handle this case appropriately
            else:
                output[i] = conv_sum / denominator
        point_array[:, 1] = output
        return point_array
        
    def gaussian_filter(self,point_array,sigma=2.0):
        input_array = point_array[:,1]
        smooth_data = gaussian_filter1d(input_array,sigma)#sigma 0.5~5.0
        point_array[:, 1] = np.round(smooth_data,3)
        return point_array
        
    def align_path(self, r_path):
        align = np.empty((0, 3))  # Initialize an empty 2D NumPy array with shape (0, 3)
    
        for i in np.arange(0.98, 1.02, 0.01):
            front = i -0.07  # car back
            back = i - 0.34  # car front
            # Extract relevant paths
            front_r_path = r_path[np.isclose(r_path[:, 2], front, atol=0.01)]
            back_r_path = r_path[np.isclose(r_path[:, 2], back, atol=0.01)]
            # Calculate right side points
            if front_r_path.size > 0 and back_r_path.size > 0:
                rf = front_r_path[:, :2].mean(axis=0)
                rb = back_r_path[:, :2].mean(axis=0)
                rf_point = [rf[0], rf[1], front]
                rb_point = [rb[0], rb[1], back]
            
                if not np.isnan(rf_point).any() and not np.isnan(rb_point).any():
                    pitch_deg, dy, dz = self.calculate_pitch(rf_point, rb_point)
                    rate = 18/27
                    height_change = -(100*dy*rate)
                    pitch_deg = pitch_deg 
                    align = np.vstack((align, [i,pitch_deg,0]))  # Append to align array
        #print("align", align)
        return align

    def create_pose(self,x, y, z, pitch_deg, roll_deg, yaw_deg):
        # Convert roll, pitch, yaw from degrees to radians
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        # Create a Pose object
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # Convert roll, pitch, yaw to a quaternion for orientation
        quaternion = quaternion_from_euler(pitch,roll, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        return pose 
        
    def serial_send(self, point):
        if point is not None:	 
            if not math.isnan(point[1]):
                self.pitch_x = round(point[1],3)
            self.y_change = point[2]
            data_to_send = f"{self. pitch_x:.1f},{self.y_change})"
            print("data_to_send:",data_to_send)
            self.ser.write(data_to_send.encode('utf-8'))
  
            
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
        all_points = np.stack((x_valid, y_valid, z_valid), axis=-1)
        
        self.points_x_minus_0_14 = all_points[np.isclose(all_points[:, 0], -0.14, atol=0.007)]#[:,0] : means all row,0 means first column
        #self.points_x_0_14 = all_points[np.isclose(all_points[:, 0], 0.14, atol=0.007)] 
        #test_time = time.time()
        averaged_points_r = self.average_y(self.points_x_minus_0_14)
        #test1_time = time.time()
        
        #print("read_data:",start_time-test_time)
        #print("average_data:",test_time-test1_time)
        
        if averaged_points_r.size > 0:
            averaged_points_r = self.gaussian_filter(averaged_points_r , 3.0)
            #test2_time = time.time()
            align_r = self.align_path(averaged_points_r)
            #test3_time = time.time()
            self.target_r = align_r[np.isclose(align_r[:, 0], 1.0, atol=0.02)]
            pitch_r = self.target_r[:, 1:3].mean(axis=0)
            self.target_1 = [1.0,pitch_r[0],pitch_r[1]]
            #print("gaussian:",test2_time-test1_time)
            #print("align:",test3_time-test2_time)
            self.publish_filtered_points(averaged_points_r)
            
            if self.ser.is_open:
                #print("Serial connection successfully established.")
                self.serial_send(self.target_1)
            else:
                print("Failed to open serial connection.")
            
        else:
            rospy.logwarn("Input points array is empty.")
        
        
        
        print("===")
if __name__ == '__main__':
    visualizer = PointCloudVisualizer()
    rospy.spin()

