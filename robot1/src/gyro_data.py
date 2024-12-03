#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import math
from sensor_msgs.msg import Imu
from scipy.ndimage import gaussian_filter1d
import numpy as np

# Global variables to store accelerometer data and calculated pitch/roll
accel_data = {"x": [], "y": [], "z": []}
pitch_data = []
roll_data = []

# Kalman Filter Class
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance  # Process noise
        self.measurement_variance = measurement_variance  # Measurement noise
        self.estimated_measurement_variance = 1.0  # Initial estimate
        self.posteri_estimate = 0.0  # Initial estimate
        self.posteri_error_estimate = 1.0  # Initial error estimate

    def update(self, measurement):
        kalman_gain = self.posteri_error_estimate / (self.posteri_error_estimate + self.measurement_variance)
        self.posteri_estimate = self.posteri_estimate + kalman_gain * (measurement - self.posteri_estimate)
        self.posteri_error_estimate = (1 - kalman_gain) * self.posteri_error_estimate + self.process_variance
        return self.posteri_estimate

# Create Kalman filters for each axis of accelerometer data
kf_x = KalmanFilter(process_variance=0.01, measurement_variance=2.0)
kf_y = KalmanFilter(process_variance=0.01, measurement_variance=2.0)
kf_z = KalmanFilter(process_variance=0.01, measurement_variance=2.0)

def accel_callback(msg):
    global accel_data

    # Extract accelerometer data
    accel_x = msg.linear_acceleration.x
    accel_y = msg.linear_acceleration.y
    accel_z = msg.linear_acceleration.z

    # Apply Kalman filter to each axis data
    filtered_x = kf_x.update(accel_x)
    filtered_y = kf_y.update(accel_y)
    filtered_z = kf_z.update(accel_z)

    # Append filtered data to the lists
    accel_data["x"].append(filtered_x)
    accel_data["y"].append(filtered_y)
    accel_data["z"].append(filtered_z)

    # Keep the data lists within a reasonable size for real-time plotting
    if len(accel_data["x"]) > 500:  # Adjust the size limit as needed
        accel_data["x"] = accel_data["x"][-500:]
        accel_data["y"] = accel_data["y"][-500:]
        accel_data["z"] = accel_data["z"][-500:]

def plot_data():
    global pitch_data, roll_data
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(10, 6))

    # Create another figure for pitch and roll data
    #fig2, ax2 = plt.subplots(figsize=(10, 6))

    while not rospy.is_shutdown():
        ax.clear()  # Clear the previous plot

        # Plot filtered accelerometer data
        ax.plot(accel_data["x"], label="Accel X (Filtered)", color="red")
        #ax.plot(accel_data["y"], label="Accel Y (Filtered)", color="green")
        #ax.plot(accel_data["z"], label="Accel Z (Filtered)", color="blue")

        # Add labels, title, and legend for accelerometer data
        ax.set_title("Accelerometer Data with Kalman")
        ax.set_xlabel("Index")
        ax.set_ylabel("Acceleration (m/s^2)")
        ax.legend()
        ax.grid()

        # Calculate pitch and roll from smoothed data (in radians)
        pitch = math.atan2(accel_data["y"][-1], accel_data["z"][-1]) + math.pi / 2
        roll = math.atan2(-accel_data["x"][-1], math.sqrt(accel_data["y"][-1]**2 + accel_data["z"][-1]**2))

        # Convert radians to degrees
        pitch_deg = math.degrees(pitch)
        roll_deg = math.degrees(roll)

        # Store the pitch and roll values in degrees for plotting
        pitch_data.append(pitch_deg)
        roll_data.append(roll_deg)
        if len(pitch_data) > 500:
            pitch_data = pitch_data[-500:]
        if len(roll_data) > 500:
            roll_data = roll_data[-500:]
            '''
        # Plot pitch and roll data in degrees
        ax2.clear()  # Clear the previous plot for pitch and roll
        ax2.plot(pitch_data, label="Pitch (°)", color="purple")
        ax2.plot(roll_data, label="Roll (°)", color="orange")

        # Add labels, title, and legend for pitch/roll data
        ax2.set_title("Pitch and Roll Data (Degrees)")
        ax2.set_xlabel("Index")
        ax2.set_ylabel("Angle (°)")
        ax2.legend()
        ax2.grid()
'''
        # Refresh both plots
        plt.draw()
        plt.pause(0.01)

def imu_listener():
    rospy.init_node('imu_accel_plotter', anonymous=True)

    # Subscribe to the accelerometer topic
    rospy.Subscriber('/camera/accel/sample', Imu, accel_callback)

    # Start the plotting function in the main thread
    plot_data()

if __name__ == '__main__':
    imu_listener()

