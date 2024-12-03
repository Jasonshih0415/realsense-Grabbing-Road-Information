#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3,PoseStamped# Import the standard message type
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from scipy.ndimage import gaussian_filter1d

# Buffer size for storing recent data for filtering
BUFFER_SIZE = 180
previous_time = None
roll = pitch = yaw = 0.0

# Initialize buffers
accel_buffer = {'x': [], 'y': [], 'z': []}
gyro_buffer = {'x': [], 'y': [], 'z': []}

def smooth_data(buffer, new_value, sigma=20):
    """Add new value to the buffer and apply Gaussian smoothing."""
    buffer.append(new_value)
    if len(buffer) > BUFFER_SIZE:
        buffer.pop(0)
    smoothed = gaussian_filter1d(buffer, sigma=sigma)
    return smoothed[-1]

def accel_callback(msg):
    global roll, pitch, accel_buffer

    # Smooth accelerometer data
    accel_x = smooth_data(accel_buffer['x'], msg.linear_acceleration.x)
    accel_y = smooth_data(accel_buffer['y'], msg.linear_acceleration.y)
    accel_z = smooth_data(accel_buffer['z'], msg.linear_acceleration.z)

    # Calculate roll and pitch using accelerometer data
    pitch = math.atan2(accel_y, accel_z) + math.pi / 2
    roll = math.atan2(-accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2))

def gyro_callback(msg):
    global yaw, previous_time, gyro_buffer

    # Smooth gyroscope data
    gyro_x = smooth_data(gyro_buffer['x'], msg.angular_velocity.x)
    gyro_y = smooth_data(gyro_buffer['y'], msg.angular_velocity.y)
    gyro_z = smooth_data(gyro_buffer['z'], msg.angular_velocity.z)

    # Get the current time
    current_time = rospy.Time.now()
    if previous_time is None:
        previous_time = current_time

    # Time difference in seconds
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time

    # Integrate gyroscope data to get yaw
    yaw_change = gyro_z * dt
    if abs(yaw_change) > 0.0001:
        yaw += yaw_change

    # Create the Vector3 message and publish it
    rpy_msg = Vector3()
    rpy_msg.x = math.degrees(pitch)
    rpy_msg.y = math.degrees(roll)
    rpy_msg.z = math.degrees(yaw)
    rpy_publisher.publish(rpy_msg)

    # Create the Pose message and set origin to (0, 0, 0)
    quat = quaternion_from_euler(pitch, 0, roll)
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
def imu_listener():
    global rpy_publisher
    global pose_publisher

    rospy.init_node('imu_orientation_calculator', anonymous=True)

    # Publisher to publish roll, pitch, and yaw data
    rpy_publisher = rospy.Publisher('/imu/rpy', Vector3, queue_size=10)
    pose_publisher = rospy.Publisher('/imu/pose', PoseStamped, queue_size=10)

    # Subscribe to accelerometer and gyroscope topics
    rospy.Subscriber('/camera/accel/sample', Imu, accel_callback)
    rospy.Subscriber('/camera/gyro/sample', Imu, gyro_callback)
    rospy.spin()

if __name__ == '__main__':
    imu_listener()
