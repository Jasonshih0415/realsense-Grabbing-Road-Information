import rospy
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import quaternion_from_euler
import math

# Initialize the ROS node
rospy.init_node("pose_array_publisher")

# Create a publisher for PoseArray messages
pose_array_pub = rospy.Publisher("pose_array_topic", PoseArray, queue_size=10)

# Define a function to create a Pose object
def create_pose(x, y, z, roll_deg, pitch_deg, yaw_deg):
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
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose

# Define an array of poses with x, y, z, roll, pitch, yaw values in degrees
poses_data = [
    {"x": 0.0, "y": 0.36, "z": 1.0, "roll": 0.0, "pitch": 0.0, "yaw": -90.0},
    {"x": 0.0, "y": 0.36, "z": 2.0, "roll": 0.0, "pitch": 0.0, "yaw": -90.0},
    {"x": 0.0, "y": 0.36, "z": 3.0, "roll": 0.0, "pitch": 0.0, "yaw": -90.0}
]

# Convert pose data to PoseArray
pose_array = PoseArray()
pose_array.header.frame_id = "camera_color_optical_frame"  # Set the appropriate frame ID
pose_array.header.stamp = rospy.Time.now()

# Add each pose to the PoseArray
for pose_data in poses_data:
    pose = create_pose(
        x=pose_data["x"],
        y=pose_data["y"],
        z=pose_data["z"],
        roll_deg=pose_data["roll"],
        pitch_deg=pose_data["pitch"],
        yaw_deg=pose_data["yaw"]
    )
    pose_array.poses.append(pose)

# Set up a rate for continuous publishing
rate = rospy.Rate(1)  # Publish at 1 Hz

# Publish the PoseArray
while not rospy.is_shutdown():
    pose_array.header.stamp = rospy.Time.now()  # Update timestamp
    pose_array_pub.publish(pose_array)
    rate.sleep()

