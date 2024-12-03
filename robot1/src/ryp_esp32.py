#!/usr/bin/env python
import rospy
import serial
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, PoseStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header

class EPS32RYP:
    def __init__(self):
        # Initialize the serial port and baud rate
        self.serial_port = '/dev/ttyUSB0'  # Update based on your system
        self.baud_rate = 115200

        # Publishers for roll, pitch, and yaw data
        self.rpy_publisher = rospy.Publisher('/imu/rpy', Vector3, queue_size=10)
        self.pose_publisher = rospy.Publisher('/imu/pose', PoseStamped, queue_size=10)

        # Initialize pitch, roll, and yaw
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.ser = None
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

    def serial_read(self):
        if self.ser and self.ser.is_open:
            try:
                response = self.ser.readline().decode('utf-8').strip()
                if response:
                    self.process_data(response)
                    print("response:",response)
            except serial.SerialException as e:
                rospy.logerr(f"Serial read error: {e}")
            except UnicodeDecodeError as e:
                rospy.logwarn(f"Data decode error: {e}")
        else:
            rospy.logerr("Serial port is not open.")
   
    def process_data(self, data):
        try:
            # Assume incoming data format: pitch,roll,yaw
            values = list(map(float, data.split(',')))
            if len(values) == 3:
                self.pitch, self.roll, self.yaw = values
            elif len(values) == 2:  # If yaw is not sent
                self.pitch, self.roll = values
                self.yaw = 0.0
            else:
                rospy.logwarn(f"Incomplete data received: {data}")
        except ValueError:
            rospy.logwarn(f"Invalid data format: {data}")

    def run(self):
        while not rospy.is_shutdown():
            loop_start = rospy.Time.now()
            
            # Prepare and publish the RPY message
            rpy_msg = Vector3()
            rpy_msg.x = self.pitch  
            rpy_msg.y = self.yaw 
            rpy_msg.z = self.roll  
            print(f"pitch:{self.pitch},yaw:{self.yaw},roll:{self.roll}")
            self.rpy_publisher.publish(rpy_msg)
            
            roll = math.radians(self.roll)
            pitch = math.radians(self.pitch)
            yaw = math.radians(self.yaw)
            # Create quaternion from roll, pitch, yaw
            quat = quaternion_from_euler(pitch, yaw, roll)

            # Prepare and publish the Pose message
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "camera_color_optical_frame"
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = 0
            pose_msg.pose.position.y = 0
            pose_msg.pose.position.z = 0
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            self.pose_publisher.publish(pose_msg)
            
            # Read serial data
            self.serial_read()



if __name__ == '__main__':
    rospy.init_node('esp32_ryp_node', anonymous=True)
    esp32_ryp = EPS32RYP()
    try:
        esp32_ryp.run()
    except rospy.ROSInterruptException:
        pass

