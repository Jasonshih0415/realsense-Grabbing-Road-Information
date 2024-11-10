# realsense-Grabbing-Road-Information

#purpose
#in this program we will use the realsense L515 depth camera to read the change in grade of the road that the car will pass over
#allowing self-driving vehicle make decision accordingly

#what should install 
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
sudo apt-get install ros-noetic-rgbd-launch # if it is needed, see hackmd situation
build catkin_workspace and put the "robot1" folder in it, remember to catkin_make to let the computer know your folder "robot1" path

#code to start camera serrvice include rgb_image,depth_image, pointcloud 
#set the picture to 640*480 to increase computing speed, and turn on IMU function, more parameter can see in launch file
roslaunch robot1 rs_rgbd.launch color_width:=640 color_height:=480 depth_width:=640 depth_height:=480 depth_fps:=30 color_fps:=30 infra_fps:=30 infra_width:=640 infra_height:=480 confidence_fps:=60 enable_gyro:=True enable_accel:=true unite_imu_method:="linear_interpolation"

#addtional function

#1_proceesing the depth_img with only get middle column of data, adjust the field of view of the camera by python, for increase computing speed
deal_depth_img.py
#2_automatically start the rviz, and it have subscribe som important topic to help us observe the data we get, if you didn't want to check the data, you can annotation it
args="-d $(find robot1)/rviz/pointcloud.rviz
#3_get IMU data, we will subscribe the accel and gcro data and turn them into roll,yaw,pitch
pub_ryp.py
#4_process the pointcloud with first get, the coordination will not ne align if we adjust the angle of camera. why we want to adjust the angle of camera is try to get most comprehensive ground data, and finally capture the road data we will facing
transfer_points.py
#5 change the pointcloud data with just camera info and depth_image, the pointcloud without color image can increase computing speed
load depth_image_proc/point_cloud_xyz
more function in realsense pointcloud can see in link:http://wiki.ros.org/depth_image_proc
