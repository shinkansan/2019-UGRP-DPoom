# Installation

## ROS
http://wiki.ros.org/kinetic/Installation/Ubuntu

## Realsense
https://github.com/IntelRealSense/realsense-ros
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

## Other pacakges
https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i


# Make custom rosbag file
I strongly recommend to record a custom rosbag file in your test environment. It is a much more efficient way to make a map file (pcd file) using __rtabmap__. 

## Publish topics of D435i
I made a custom launch file to publish essential topics of realsense camera D435i, named __'tk_rosbag_realsense_for_mapping.launch'.__ You must run this launch file before record custom rosbag file. 

Source: [tk_rosbag_realsense_for_mapping.launch](tk_rosbag_realsense_for_mapping.launch) .

```bash
$ roslaunch realsense2_camera tk_rosbag_realsense_for_mapping.launch
```

## Record rosbag file
You can record your rosbag file by run scripts below:

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
