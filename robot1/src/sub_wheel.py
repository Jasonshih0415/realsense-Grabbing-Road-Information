import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

class PointCloudVisualizer:
    def __init__(self):
        self.points = np.empty((0, 3))  # Initialize with an empty array

        # Initialize the ROS node
        rospy.init_node('pointcloud_visualizer', anonymous=True)

        # Subscribe to the filtered point cloud topic
        rospy.Subscriber('/filtered_points_transform', PointCloud2, self.callback_pointcloud)

        # Setup matplotlib
        self.fig, self.ax = plt.subplots()
        self.scatter = self.ax.scatter([], [])
        self.ax.set_xlim(0, 3)  # Adjust limits based on your data
        self.ax.set_ylim(-1, 0)  # Adjust limits based on your data
        self.ax.set_xlabel('Distance (z)')
        self.ax.set_ylabel('Height (y)')

        # Start the animation
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, frames=self.get_data, interval=100)

    def callback_pointcloud(self, data):
        # Convert PointCloud2 to numpy array
        points = np.fromiter(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True), 
                             dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        # Change data to (N, 3)
        self.points = np.round(np.vstack((points['x'], points['y'], points['z'])).T, 2)

    def get_data(self):
        while not rospy.is_shutdown():
            yield self.points

    def average_ground_points(self, points, height_threshold=0.1):
        # Find unique z values
        unique_z = np.unique(points[:, 2])
        averaged_data = []

        for z in unique_z:
            # Get rows where z is equal to the current z value
            rows_with_same_z = points[points[:, 2] == z]

            # Calculate the average x and y for ground points
            mean_xy = rows_with_same_z[:, :2].mean(axis=0)
            averaged_row = np.array([mean_xy[0], mean_xy[1], z])

            averaged_data.append(averaged_row)

        return np.array(averaged_data)


    def align_path(self, r_path, l_path):
        unique_zr = np.unique(r_path[:, 2])
        unique_zl = np.unique(l_path[:, 2])
        union_z = np.union1d(unique_zr, unique_zl)
        
        aligned_points = []

        for z in union_z:
            # Check if the z value exists in r_path and l_path
            r_match = r_path[r_path[:, 2] == z]  # Points in r_path with z value equal to current z
            l_match = l_path[l_path[:, 2] == z]  # Points in l_path with z value equal to current z

            if r_match.size > 0:
                y_r = r_match[0, 1]  # Get the y value from the first row that matches (since all rows would have the same z)
            else:
                y_r = None  # Indicate no matching point found in r_path

            if l_match.size > 0:
                y_l = l_match[0, 1]  # Get the y value from the first row that matches
            else:
                y_l = None  # Indicate no matching point found in l_path

            # Add the aligned z, y_r, and y_l to the results list
            aligned_points.append((z, y_r, y_l))

        # Convert the aligned points list to a NumPy array
        return np.array(aligned_points, dtype=[('z', float), ('y_r', float), ('y_l', float)])

    def update_plot(self, points):
        if points is not None and points.size > 0:
            # Filter points where x is close to 0.14 (within a small tolerance to account for float precision)
            self.points_x_minus_0_14 = points[np.isclose(points[:, 0], 0.14, atol=0.01)]
            self.points_x_0_14 = points[np.isclose(points[:, 0], -0.14, atol=0.01)]
            
            if self.points_x_minus_0_14.size > 0:
                # Get the averaged ground points based on z values
                averaged_points_r = self.average_ground_points(self.points_x_minus_0_14)
                averaged_points_l = self.average_ground_points(self.points_x_0_14)
                aligned_points = self.align_path(averaged_points_r, averaged_points_l)
                print("points_r:",averaged_points_r.shape)
                print("points_l:",averaged_points_l.shape)
                print("align.len:",aligned_points.shape)
                z_values = aligned_points['z']  # Get z values
                y_r_values = -aligned_points['y_l']  # Get y_r values (inverted for display)

                # Update scatter plot
                self.scatter.set_offsets(np.c_[z_values, y_r_values])
            else:
                self.scatter.set_offsets(np.empty((0, 2)))  # Clear if no points meet the criteria
                if hasattr(self, 'regression_line'):
                    self.regression_line.set_data([], [])  # Clear the regression line if no points available


if __name__ == '__main__':
    visualizer = PointCloudVisualizer()
    plt.show()
    rospy.spin()

