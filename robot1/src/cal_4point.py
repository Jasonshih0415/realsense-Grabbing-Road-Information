import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import time

class PointCloudVisualizer:
    def __init__(self):
        self.points_x_minus_0_14 = np.empty((0, 3))  # Initialize with an empty array
        self.points_x_0_14 = np.empty((0, 3))        # Initialize the other array

        # Initialize the ROS node
        rospy.init_node('cal_4points', anonymous=True)

        # Subscribe to the filtered point cloud topic
        rospy.Subscriber('/filtered_points_transform', PointCloud2, self.callback_pointcloud)

    def average_ground_points(self, points, height_threshold=0.1):
        # Find unique z values
        #print("points.shape:",points.shape)
        unique_z = np.unique(points[:, 2])
        averaged_data = []

        for z in unique_z:
            # Get rows where z is equal to the current z value
            rows_with_same_z = points[points[:, 2] == z]
            mean_xy = rows_with_same_z[:, :2].mean(axis=0)
            averaged_row = np.array([mean_xy[0], mean_xy[1], z])
            averaged_data.append(averaged_row)
        return np.array(averaged_data)
        
    def calculate_roll_pitch(self,pillars):
        # Create two vectors from the remaining three points
        v1 = pillars[1] - pillars[0]
        v2 = pillars[2] - pillars[0]
        #print('v1',v1)
        #print('v2',v2)

        # Compute the normal vector to the plane using the cross product
        normal_vector = np.cross(v1, v2)
        #print("normal_vector",normal_vector)
        A, B, C = normal_vector

        # Calculate roll and pitch
        roll = np.arctan2(B, C)  # Roll (theta_x) in radians
        pitch = np.arctan2(-A, np.sqrt(B**2 + C**2))  # Pitch (theta_y) in radians

        # Convert to degrees
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        #print('roll',roll_deg)
        #print('pitch',pitch_deg)
        return roll_deg, pitch_deg
        
    def align_path(self, r_path, l_path):
        align=[]
        for i in np.arange(0,2,0.01):
            #print("i:",i)
            front = i+0.14
            back = i-0.14
            r_f = r_path[r_path[:, 2] == front] #if not detect, return empty array
            r_b = r_path[r_path[:, 2] == back] 
            l_f = l_path[l_path[:, 2] == front] 
            l_b = l_path[l_path[:, 2] == back]                   
            arrays_to_check = [r_f, r_b, l_f, l_b]
            pillars = np.array([arr for arr in arrays_to_check if arr.size > 0], dtype=float) #append if the array is not empty (num, 1,3)
            pillars = pillars.reshape(-1, 3)
            if pillars.shape[0] == 3:
                roll_deg, pitch_deg=self.calculate_roll_pitch(pillars)
                #print("3 point:",pillars)
                #print("roll:",roll_deg)
                #print("pitch:",pitch_deg)
                align.append([i,roll_deg,pitch_deg])
            elif pillars.shape[0] == 4:
                #points = pillars.reshape(-1, 3)
                min_z_index = np.argmin(pillars[:, 2])
                points_d = np.delete(pillars, min_z_index, axis=0) #delete the points with smallest z
                roll_deg, pitch_deg=self.calculate_roll_pitch(pillars)
                #print("4 point:",points_d)
                #print("roll:",roll_deg)
                #print("pitch:",pitch_deg)
                align.append([i,roll_deg,pitch_deg])
        print("align",len(align))
        print("align",align)
        #return align
        
    
    def callback_pointcloud(self, data):
        # Convert PointCloud2 to numpy array
        points = np.fromiter(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True), 
                             dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        # Change data to (N, 3)
        all_points = np.round(np.vstack((points['x'], points['y'], points['z'])).T, 2)
        # Separate points based on x = -0.14 and 0.14 with tolerance for floating point precision
        self.points_x_minus_0_14 = all_points[np.isclose(all_points[:, 0], -0.14, atol=0.01)]#[:,0] : means all row,0 means first column
        self.points_x_0_14 = all_points[np.isclose(all_points[:, 0], 0.14, atol=0.01)]

        # Timing calculations
        start_time = time.time()
        averaged_points_r = self.average_ground_points(self.points_x_minus_0_14)
        averaged_points_l = self.average_ground_points(self.points_x_0_14)
        #print("type:",type(averaged_points_r))
        self.align_path(averaged_points_r ,averaged_points_l)
        #print("align.shape",len(align))
        #print("align",align) 
        averaged_time = time.time()
        
        print("===")
if __name__ == '__main__':
    visualizer = PointCloudVisualizer()
    rospy.spin()

