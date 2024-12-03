import rospy
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
import numpy as np

def calculate_vectors(pillars):
    # Calculate vectors in the plane
    v1 = pillars[1] - pillars[0]
    v2 = pillars[2] - pillars[0]
    
    # Calculate normal vector using cross product
    normal_vector = np.cross(v1, v2)
    A, B, C = normal_vector

    # Calculate roll and pitch based on normal vector
    roll = np.arctan2(B, C)
    pitch = np.arctan2(-A, np.sqrt(B**2 + C**2))
    roll_deg = np.degrees(roll)
    pitch_deg = np.degrees(pitch)
    print("v1",v1)
    print("v2",v2)
    print("normal_vector",normal_vector)
    print("roll",roll)
    print("pitch",pitch)
    print("roll_deg ",roll_deg )
    print("pitch_deg ",pitch_deg )
    
    
    return v1, v2, normal_vector, roll, pitch

def create_marker(vector, start_point, marker_id, color):
    # Creates a Marker of type ARROW to represent a vector in RViz
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.id = marker_id
    marker.scale.x = 0.1  # Shaft diameter
    marker.scale.y = 0.2  # Head diameter
    marker.scale.z = 0.2  # Head length

    # Set the color of the marker
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    # Define the start and end points of the arrow
    marker.points = [Point(start_point[0], start_point[1], start_point[2]),
                     Point(start_point[0] + vector[0], 
                           start_point[1] + vector[1], 
                           start_point[2] + vector[2])]
    
    return marker

def publish_markers(pillars, v1, v2, normal_vector):
    rospy.init_node('vector_visualizer')
    marker_pub = rospy.Publisher('camera_color_optical_frame', Marker, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Create markers for v1, v2, and normal_vector
        marker_v1 = create_marker(v1, pillars[0], 0, (1.0, 0.0, 0.0))  # Red for v1
        marker_v2 = create_marker(v2, pillars[0], 1, (0.0, 1.0, 0.0))  # Green for v2
        marker_normal = create_marker(normal_vector, pillars[0], 2, (0.0, 0.0, 1.0))  # Blue for normal vector

        # Publish markers
        marker_pub.publish(marker_v1)
        marker_pub.publish(marker_v2)
        marker_pub.publish(marker_normal)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        # Define the points in the plane
        pillars = np.array([
            [-0.14, 0.05, 1.21],  # Point 1
            [0.14, 0.04, 1.21],  # Point 2
            [0.14, 0.04, 1.19]   # Point 3
        ])
        
        # Calculate vectors and orientation
        v1, v2, normal_vector, roll, pitch = calculate_vectors(pillars)
        
        # Publish the vectors as markers in RViz
        publish_markers(pillars, v1, v2, normal_vector)

    except rospy.ROSInterruptException:
        pass

