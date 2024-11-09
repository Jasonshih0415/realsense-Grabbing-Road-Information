#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3  # Import the standard message type

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
    pitch = math.atan2(accel_y, accel_z)
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

    # Integrate gyroscope data to get yaw
    yaw += gyro_z * dt

    # Create the Vector3 message and publish it
    rpy_msg = Vector3()
    rpy_msg.x = math.degrees(pitch)
    rpy_msg.y = math.degrees(roll)
    rpy_msg.z = math.degrees(yaw)
    rpy_publisher.publish(rpy_msg)

    # Optional: Print roll, pitch, and yaw for debugging
    #rospy.loginfo("Pitch: {:.2f}, Roll: {:.2f}, Yaw: {:.2f}".format(rpy_msg.x, rpy_msg.y, rpy_msg.z))

def imu_listener():
    global rpy_publisher

    rospy.init_node('imu_orientation_calculator', anonymous=True)

    # Publisher to publish roll, pitch, and yaw data
    rpy_publisher = rospy.Publisher('/imu/rpy', Vector3, queue_size=10)

    # Subscribe to accelerometer and gyroscope topics
    rospy.Subscriber('/camera/accel/sample', Imu, accel_callback)
    rospy.Subscriber('/camera/gyro/sample', Imu, gyro_callback)
    rospy.spin()

if __name__ == '__main__':
    imu_listener()

