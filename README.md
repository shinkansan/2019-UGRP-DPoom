Indoor Mobile Robot Fully Autonomous Driving Project - DPoom ![build_status](https://img.shields.io/badge/build-WIP-yellow.svg)
==============================
For further information, please visit our [page](https://shinkansan.github.io/2019-UGRP-DPoom/)

Click below for introduction video.
<div align="center">
  <a href="https://www.youtube.com/watch?v=9KRy7pCXqaM&feature=youtu.be"><img src="https://img.youtube.com/vi/9KRy7pCXqaM/0.jpg" alt="Introduction Video"></a>
</div>

### Single board computer Lattepanda Alpha based indoor mobile robot DPoom, with fully autonomous driving system using single RGB-D camera Intel Realsense D435i. 

> Keywards: autonomy, autonomous driving system, mobile robot, SLAM, ROS, RGB-D, Low-end, global path planning, motion planning, ground segmentaion, navigation, path tracking, control, Human-Robot Interaction


# Installation
### [Follow installation guide here](installation_guide)


# How to Run
## Robot control
The robot control two dynamixel motors via ROS and OpenCR. For providing a easy way of robot control, we built a driving control package name easyGo. See [Control Package Page](easygo/REAME.md).
## SLAM
Mapping should be preceded before deploying robots. See [SLAM Page](SLAM/README.md).
## Global path planning
Using pcd map, the robot can plan global path using our FMM based modified A*. See [GPP Page](pathplanning/README.md).
## Motion planning
The robot can follow the generated path by motion planner. Our motion planner is using our real-time ground segmentation method named MORP. See [Motion Planning (MORP) Page](MORP/README.md).
## Full autonomous driving
RGB-D localization, global path planning and motion planning are integrated in one python script. Just run [integration.py](integration.py). For details, see [Integration Page](autodrive.md)
## Human-Computer Interaction
See [Human-Computer Interaction Page](HCI/README.md).

# Related Repositories
## DPoom_Gazebo
DPoom is also availble in ROS Gazebo simulation with __equivalent__ codes. To simulate DPoom in Gazebo:  [DPoom_gazebo](https://github.com/SeunghyunLim/Dpoom_gazebo)
## Gazebo-CrowdNav
Our navigation method can be simulated in Gazebo. Current state-of-the-art navigation approches based on DRL ([CADRL](https://ieeexplore.ieee.org/abstract/document/7989037), [LSTM-RL](https://ieeexplore.ieee.org/abstract/document/8593871), [SARL](https://ieeexplore.ieee.org/abstract/document/8794134)) are available with DPoom platform. To evalute navigation performance with DPoom in Gazebo: [Gazebo-CrowdNav](https://github.com/ktk1501/Gazebo-CrowdNav)

# Project Info
>__2019/12/5 We opened our github repos to public!!.__

| Without distance cost | With distance cost |
|---|---|
|![a](https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/docs/gif/DPoom_temp.gif)|![a](https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/docs/gif/MORP_test.gif)|

## Purpose of this project
These days mobile robots are rapidly developing
in the industry. However, there are still some problems for
practical application such as expensive hardware and high
power consumption. In this study, we propose a navigation
system, which can be operated on a low-end computer with
an RGB-D camera, and a mobile robot platform to operate
integrated autonomous driving system. The proposed system
does not require LiDARS or GPUs. Our raw depth image
ground segmentation extracts a traversability map for safe
driving of the low-body mobile robots. It is designed to
guarantee real-time performance on a low-cost commercial
single board computer with integrated SLAM, global path
planning, and motion planning. Running sensor data processing
and other autonomous driving functions simultaneously, our
navigation method performs fast at 18Hz refresh rate for
control command, while the others have slower refresh rates.
Our method outperforms current state-of-the-art navigation
approaches as shown in 3D simulation tests. In addition, we
demonstrate the applicability of our mobile robot system by
successfully autonomous driving in a residential lobby.

## Our team
Taekyung Kim  / [DGIST](https://www.dgist.ac.kr/kr/introen2020.html) Class of 2020 @ktk1501<br/>
Seunghyun Lim / DGIST Class of 2020 @SeunghyunLim<br/>
Gwanjun Shin  / DGIST undergraduate @shinkansan<br/>
Geonhee Sim   / DGIST undergraduate @jane79<br/>

## Platform info
### SOFTWARE
- Ubuntu 16.04 LTS
- ROS Kinetic
- Python 3.6
- Python 2.7 on Robot Platform
- Realsense SDK 2
- Tensorflow 1.8.0
- OpenCV
### HARDWARE
 - Platform Computing Unit : Lattepanda Alpha 864
 - Intel Realsense Camera D435i
 - Turtlebot3 waffle pi
 - Outer HW printed by 3D-Printer

 ## Paper info
 Our paper had submitted to [IROS 2021](https://www.iros2021.org/)

#### Build Status Badge by [shield.io](https://shields.io/category/build)
#### This work is partially supported by DGIST UGRP.