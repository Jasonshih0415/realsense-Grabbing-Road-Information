#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import math
from sensor_msgs.msg import Imu
from scipy.ndimage import gaussian_filter1d

# Global variables to store accelerometer data and calculated pitch/roll
accel_data = {"x": [], "y": [], "z": []}
pitch_data = []
roll_data = []

def accel_callback(msg):
    global accel_data

    # Extract accelerometer data
    accel_x = msg.linear_acceleration.x
    accel_y = msg.linear_acceleration.y
    accel_z = msg.linear_acceleration.z

    # Append data to the lists
    accel_data["x"].append(accel_x)
    accel_data["y"].append(accel_y)
    accel_data["z"].append(accel_z)

    # Keep the data lists within a reasonable size for real-time plotting
    if len(accel_data["x"]) > 500:  # Adjust the size limit as needed
        accel_data["x"] = accel_data["x"][-500:]
        accel_data["y"] = accel_data["y"][-500:]
        accel_data["z"] = accel_data["z"][-500:]

def plot_data():
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(10, 6))

    # Create another figure for pitch and roll data
    fig2, ax2 = plt.subplots(figsize=(10, 6))

    while not rospy.is_shutdown():
        ax.clear()  # Clear the previous plot

        # Apply Gaussian filter to smooth the accelerometer data
        smoothed_x = gaussian_filter1d(accel_data["x"], sigma=10)
        smoothed_y = gaussian_filter1d(accel_data["y"], sigma=10)
        smoothed_z = gaussian_filter1d(accel_data["z"], sigma=10)

        # Calculate pitch and roll from smoothed data (in radians)
        pitch = math.atan2(smoothed_y[-1], smoothed_z[-1]) + math.pi / 2
        roll = math.atan2(-smoothed_x[-1], math.sqrt(smoothed_y[-1]**2 + smoothed_z[-1]**2))

        # Convert radians to degrees
        pitch_deg = math.degrees(pitch)
        roll_deg = math.degrees(roll)

        # Store the pitch and roll values in degrees for plotting
        pitch_data.append(pitch_deg)
        roll_data.append(roll_deg)

        # Plot smoothed accelerometer data
        ax.plot(smoothed_x, label="Accel X (Smoothed)", color="red")
        #ax.plot(smoothed_y, label="Accel Y (Smoothed)", color="green")
        #ax.plot(smoothed_z, label="Accel Z (Smoothed)", color="blue")

        # Add labels, title, and legend for accelerometer data
        ax.set_title("Accelerometer Data with gaussian")
        ax.set_xlabel("Index")
        ax.set_ylabel("Acceleration (m/s^2)")
        ax.legend()
        ax.grid()
        
        # Plot pitch and roll data in degrees
        ax2.clear()  # Clear the previous plot for pitch and roll
        ax2.plot(pitch_data, label="Pitch (°)", color="purple")
        #ax2.plot(roll_data, label="Roll (°)", color="orange")

        # Add labels, title, and legend for pitch/roll data
        ax2.set_title("Pitch and Roll Data (Degrees)")
        ax2.set_xlabel("Index")
        ax2.set_ylabel("Angle (°)")
        ax2.legend()
        ax2.grid()

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

