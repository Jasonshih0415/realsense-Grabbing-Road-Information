#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3,PoseStamped# Import the standard message type
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header

# Initialize global variables
roll = pitch = yaw = 0.0
previous_time = None

def accel_callback(msg):
    global roll, pitch

    # Extract accelerometer data
    accel_x = msg.linear_acceleration.x
    accel_y = msg.linear_acceleration.y
    accel_z = msg.linear_acceleration.z

    # Calculate roll and pitch using accelerometer data
    pitch = math.atan2(accel_y, accel_z)+math.pi/2
    roll = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

def gyro_callback(msg):
    global yaw, previous_time

    # Get the current time
    current_time = rospy.Time.now()
    if previous_time is None:
        previous_time = current_time

    # Time difference in seconds
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time

    # Extract gyroscope data
    gyro_z = msg.angular_velocity.z
    gyro_x = msg.angular_velocity.x
    gyro_y = msg.angular_velocity.y
    
    # Integrate gyroscope data to get yaw
    yaw_change = gyro_z * dt
    #print("yaw_change ",yaw_change )
    if abs(yaw_change) >0.0001:
        yaw += yaw_change 

    # Create the Vector3 message and publish it
    rpy_msg = Vector3()
    rpy_msg.x = math.degrees(pitch)
    rpy_msg.y = math.degrees(roll)
    rpy_msg.z = math.degrees(yaw)
    rpy_publisher.publish(rpy_msg)
    
    """
    quat = quaternion_from_euler(pitch, 0, roll)
    # Create the Pose message and set origin to (0, 0, 0)
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

    pose_publisher.publish(pose_msg)
    
    """
    # Optional: Print roll, pitch, and yaw for debugging
    #rospy.loginfo("Pitch: {:.2f}, Roll: {:.2f}, Yaw: {:.2f}".format(rpy_msg.x, rpy_msg.y, rpy_msg.z))
    
def imu_listener():
    global rpy_publisher
    global pose_publisher

    rospy.init_node('imu_orientation_calculator', anonymous=True)

    # Publisher to publish roll, pitch, and yaw data
    rpy_publisher = rospy.Publisher('/imu/rpy_test', Vector3, queue_size=10)
    #pose_publisher = rospy.Publisher('/imu/pose', PoseStamped, queue_size=10)

    # Subscribe to accelerometer and gyroscope topics
    rospy.Subscriber('/camera/accel/sample', Imu, accel_callback)
    rospy.Subscriber('/camera/gyro/sample', Imu, gyro_callback)
    rospy.spin()

if __name__ == '__main__':
    imu_listener()

