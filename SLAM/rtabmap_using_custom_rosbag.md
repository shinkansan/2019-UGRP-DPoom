# Make custom rosbag file
__I strongly do not recommend to record a custom rosbag file to test rtabmap !!!!!!!!__ It may be more efficient way to make a map file (pcd file). However, rtabmap gives you a chance to restore odometry when it fails tracking, by relocating the camera to the last tracked position. __So I recommend to make a map by online method, not by using rosbag file!!__

## Publish topics of D435i
I made a custom launch file to publish essential topics of realsense camera D435i, named __'tk_rosbag_realsense_for_mapping.launch'.__ You must run this launch file before record custom rosbag file. 

Source: [tk_rosbag_realsense_for_mapping.launch](tk_rosbag_realsense_for_mapping.launch) .

```bash
$ roslaunch realsense2_camera tk_rosbag_realsense_for_mapping.launch
```

## Record rosbag file
You can record your rosbag file by running scripts below:

```bash
$ rosparam set use_sim_time false
$ rosbag record -O my_bagfile_1.bag /camera/aligned_depth_to_color/camera_info  camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/color/image_raw /camera/imu /camera/imu_info /tf_static
```
You can also change name of the rosbag file by changing 'my_bagfile_1.bag'. It must includes all the topics above to run rtabmap.

## Check rosbag info
To check your rosbaf file is vaild, run scripts below:

```bash
$ rosbag info my_bagfile_1.bag
```
The results should be like this:
```bash
topics:      /camera/aligned_depth_to_color/camera_info     4057 msgs    : sensor_msgs/CameraInfo
             /camera/color/camera_info                      4155 msgs    : sensor_msgs/CameraInfo
             /camera/color/image_raw                        4155 msgs    : sensor_msgs/Image     
             /camera/imu                                  108676 msgs    : sensor_msgs/Imu       
             /tf_static                                        1 msg     : tf2_msgs/TFMessage    
             camera/aligned_depth_to_color/image_raw        4057 msgs    : sensor_msgs/Image
```


## Run your rosbag file
```bash
roscore >/dev/null 2>&1 &
rosparam set use_sim_time true
rosbag play my_bagfile_1.bag --clock
```
After play rosbag file, run scripts below to start __'rtabmap'__ to listen topics from your rosbag file, to perform mapping.

```bash
roslaunch realsense2_camera opensource_tracking.launch offline:=true
```

## Save your pointcloud map to pcd
You can save your lovely pointcloud map to pcd using:
```bash
rosrun pcl_ros pointcloud_to_pcd input:=/rtabmap/cloud_map
```
Be care that you should run it before rosbag file finishes. The app will display "Data saved to xxxxxx.pcd". 

# Change Parameters

## change minimum inlier size. 
The main problem of mapping custom rosbag file was losing tracking of odometry from IMU. I can see a lot of warning message as below: 
```bash
OdometryF2M.cpp:469::computeTransform() Registration failed: "Not enough inliers 0/20 (matches=5) between -1 and 795"
```

I am trying to decrease __'minimum inlier size'. I think it will be more flexible to accept quickly rotating IMU.__ I earned hint from here:
http://answers.ros.org/question/267741/problem-with-stereo-outdoor-mapping-using-rtabmap_ros/

It's a generally used launch file that change some parameters in rtabmap. In __rtabmap.launch__, there are such instructions:

```bash
<include file="$(find rtabmap_ros)/launch/rtabmap.launch">
    <arg name="frame_id" value="base_footprint"/>
    <arg name="stereo" value="true"/>
    <arg name="approx_sync" value="true"/>
    <arg name="left_image_topic"        value="/stereo_camera/left/image_rect_color" />
    <arg name="right_image_topic"       value="/stereo_camera/right/image_rect" /> 
    <arg name="left_camera_info_topic"  value="/stereo_camera/left/camera_info_throttle" />
    <arg name="right_camera_info_topic" value="/stereo_camera/right/camera_info_throttle" />
    <arg name="rtabmap_args" value="--delete_db_on_start --Vis/EstimationType 1 --Vis/MaxDepth 0 --GFTT/QualityLevel 0.00001 --Stereo/MinDisparity 0 --Stereo/MaxDisparity 64 --Vis/RoiRatios '0 0 0 .2' --Kp/RoiRatios '0 0 0 .2' --Odom/GuessMotion true --Vis/MinInliers 10 --Vis/BundleAdjustment 1 --OdomF2M/BundleAdjustment 1 --Vis/CorNNDR 0.6 --Vis/CorGuessWinSize 20 --Vis/PnPFlags 0"/>
```
You should change parameters such as  __'--Vis/MinInliers 10'__, by adding arguments setting in my rtabmap launch file. Default MinInliers is 20. 

## Important Parameters Here
### MinInliers
Decrease it to be more flexible to IMU changes. Changed it '20 -> 15'.
### LoopClosureReextractFeatures
Set this to true to perform loop closure. Changed it 'false -> true'.
### approx_sync
Set this to false to not let 'rtabmap' skip some topics from rosbag file. Changed it 'true -> false'.
### queue_size
Increase it dramatically to queue more rosbag topics. Since I don't want 'rtabmap' to skip any topics because of computation delay while mapping. Changed it '10 -> 5000'. 
### wait_for_transform
Increase it to let 'rtabmap' to wait for more time, for the computation delay of transform. Changed it '0.1 -> 1.0'

## Modified 'opensource_tracking.launch' file
I changed the important parameters above in 'opensource_tracking.launch' file. Changed parts can be seen below:
```bash
<include file="$(find rtabmap_ros)/launch/rtabmap.launch">
		<arg name="rtabmap_args" value="
      --delete_db_on_start
      --RGBD/LoopClosureReextractFeatures true
      --Vis/MinInliers 15
"/>
	
		<arg name="approx_sync" value="false"/>
    <arg name="queue_size" value="5000"/>
	  <arg name="wait_for_transform" value="1.0"/>

	  <arg name="args" value="--delete_db_on_start "/>
    <arg name="rgb_topic" value="/camera/color/image_raw"/>
    <arg name="depth_topic" value="/camera/aligned_depth_to_color/image_raw"/>
    <arg name="camera_info_topic" value="/camera/color/camera_info"/>
    <arg name="depth_camera_info_topic" value="/camera/depth/camera_info"/>
    <arg name="rtabmapviz" value="false"/>
    <arg name="rviz" value="true"/>
	</include>
```
Source file are uploaded here: [opensource_tracking_tk_parameter_2.launch](opensource_tracking_tk_parameter_2.launch)

# Run rtabmap to SLAM with D435i
The instructions referenced this site: https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i

1. Play rosbag file you recorded. I record a bag file in DGIST E5 2nd floor, and I named it 'my_bagfile_4.bag'. It was over 9GB. 
(Be sure to set 'use_sim_time' as false, by 'rosparam set use_sim_time true')
```bash
$ rosbag play my_bagfile_4.bag --clock
```
2. Run 'rtabmap' by launch 'opensource_tracking.launch' in 'realsense2_camera' package. I used my custom launch file. RVIZ will be automatically ran. Pray for your rosbag file is a well-made data.
```bash
$ roslaunch realsense2_camera opensource_tracking_tk_parameter_2.launch offline:=true
```
3. Save your lovely pointcloud map to pcd using:
```bash
rosrun pcl_ros pointcloud_to_pcd input:=/rtabmap/cloud_map
```
Be care that you should run it before rosbag file finishes. The app will display "Data saved to xxxxxx.pcd". 

# Results
## Loop Closure
To be updated.

## PCD file
To be updated.

# Possible Issue
I always lose odometry in the same position, no matter how I carefully changed its parameters. The problem was that, in my rosbag file, there is one moment that captures a uniform white wall without any other objects. 'rtabmap' uses imu data in '/camera/imu' topics, but it is only referenced by odometry, and matching priority of 'rtabmap' is not odometry. When there are few features to match, which means less than MinInliers, it loses tracking odometry.
I recorded a new rosbag file that does not contain any moments capturing all white wall. Then I succeeded to perform mapping!
