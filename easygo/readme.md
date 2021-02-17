ROS easyGo and ros docs
===

## ROS Document for basic rospy system

[ROS catkin_pkg init](https://github.com/LCAS/teaching/wiki/First-Turtlebot-coding)

[ROS Turtlebot Offcial Docs](http://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)

[ROS Optional HOW TO CHANGE OpenCR Port](http://wiki.ros.org/rosserial_python)

[ROS Lecture Github wikim Great explanations](https://github.com/LCAS/teaching/wiki/CMP3103M#week-1-first-steps)

[ROS, Using ROSPY & Make subscriber method ](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)

[ROS, Example Code Get Image from Publisher](https://github.com/LCAS/teaching/blob/kinetic/cmp3103m-code-fragments/scripts/opencv_bridge.py)

[ROS, cmd_vel example](http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right)

[Korean, About cmd_vel](http://blog.naver.com/PostView.nhn?blogId=dolmangi&logNo=220530436434&beginTime=0&jumpingVid=&from=section&redirect=Log&widgetTypeCall=true)

[ROS Example code for subscriber](https://answers.ros.org/question/131976/subscribing-and-publishing-to-a-twist-message/)

[Publisher to a topic](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)

# Dependencies
* ROS Kinetic
* turtlebot3 : http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#pc-setup

## How to catkin_make
you should make catking_ws folder right after make files in catkin_ws and its sub directories


__But!! Before catkin_make__
you must give a mod (a+x) by chmod,
it makes file executable
```bash
chmod a+x {your_file}
```
Once you did this .py file
you don't have to do it each times

and Catkin_make!
```bash
# make source file
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
```

__Or if you want temporally run just for test you can just run this py file by normal way__
```bash
python {your_file.py}
```

and run your py file via rosrun
```bash
# don't type {} into actual command
rosrun {your catkin_pkg name} {your_sourcefile.py}
```

## Start ROS Environment
```bash
roscore #it make lattepanda work as hub
roslaunch turtlebot3_bringup turtlebot3_core.launch #it link a driver like openCR to lattepanda

```


## Actual Test [Real robot needed]
To start easygo.py

1. make sure openCR is on
2. do __Start ROS Environment__ section
3. if you want run easygo or your py file, for test. just run it by python command
4. Checkout catkin_pkg make tutorial from above docs

# easyGo ![build badge](https://img.shields.io/badge/build-passing-green.svg)


Source: [./easyGo.py](./easyGo.py)


you can import this by
```bash
#if your py file and easygo file in same dir
from easyGo import easygo
```

### Stop
```bash
easyGo.stop(pub, verbose=0)
#Stop All Vector (x,y,z)
```
## Verbose ?
Verbose = 0 (default) Don;t print status
| Verbose = 1 Print Everything

### Rotate
```bash
easyGo.mvRotate(speed, angle, clockwise, verbose=0)
'''Rotate {0} degree with {1} degree/sec Clockwise = {2}'''
```

if angle=-1, turn CW/CCW continuously</br>


### Forward/Backward
```python
easyGo.mvStraight(speed, angle, verbose=0)
#Stop All Vector (x,y,z)
```
angle = -1 is for inf go, the example code is below


### Curve with linear, angular Value
#### __mvCurve__
```Python
easyGo.mvCurve(speed, steer)
#speed for linear velocity {negatives for reverse, positives for forward (float)}
#steer for float value for angular speed
```
_you can check the example of this func by easyVector section_

### function example
```python



if __name__ == '__main__':
    try:
        rospy.init_node('robot_goForward', anonymous=True)
        velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        speed = float(input("Input your speed (degrees/sec):"))
        angle = float(input("Type your distance (degrees):"))
        clockwise = input("Clockwise?: ") #True or false
        # Testing our function

        #rotate(speed, angle, clockwise)

        #Verbose = 0 (default) Don;t print status
        #Verbose = 1 Print Everything


        '''   Infinity go Example ..........break condition DIY
        t0 = rospy.Time.now().to_sec()
        while True:
            mvStraight(velocity_publisher, speed, angle, 1)
            t1 = rospy.Time.now().to_sec()
            if t1-t0 > 10:
                break
        stop()
        '''       



    except rospy.ROSInterruptException:
        pass
```


# easyControl ![build badge](https://img.shields.io/badge/Version-beta-yellow.svg)
go to source [./easyControl.py](./easyControl.py)

easyControl is for operating turtlebot by keyboard input

it should be working which keyCap.py

## Basic keyset
  w : go forward</br>
  a : turn counter clockwise</br>
  s : go backward</br>
  d : turn clockwise</br>
  q : stop & terminate program</br>
  x : set default speed</br>
  e : speed up by 0.1</br>
  c : speed down by 0.2</br>

Any keyevent which is not on the above is for e-stop



# easyVector ![build badge](https://img.shields.io/badge/Version-beta-yellow.svg)
go to source [./easyVector.py](./easyVector.py)<br/>
easyVector is IMU Based Steer Assistant solution and Odom Pose X Y Provider

it can be run by standalone or run by import<br/>
> __IMPORTANT!!__ <br/>
If you wanna run it by import
you must Initialize ros node top of your code <br/>
```python
import rospy
rospy.init_node('robot_mvs', anonymous=True)
```

  After that just implement easyVector by ```import easyVector```

## Usage, Steer Value
```python
easyVector.get_steer_Value(desired_angle)

easyVector.get_poseXY()
```
and this ```get_steer_Value(@params)``` returns steer value for ```easyGo.mvCurve(speed, steer)``` <br/>
```get_poseXY()``` returns PoseXY as tuple (poseX, poseY) in meter

# imu2angle ![build badge](https://img.shields.io/badge/build-passing-green.svg)

imu Yaw value to 'imu_raw' topics on ROSPY



# DAP (Drive Along Path) ![build badge](https://img.shields.io/badge/Version-beta-yellow.svg)


go to source [./dap.py](./dap.py)<br/>
dap let DPoom drive along given path

it can be run by standalone or run as library (path & Robot_speed are required)
it should run with easySeries


you can import this by

```python
#if your py file and dap file in same dir
import dap
```

## Dependency
```python
import rospy
import easyGo
import easyVector
import math
import matplotlib.pyplot as plt
import cv2 # if you want to see visualized steer value(show what direction should DPoom head to)
```


## Main functions
#### 1. Steer_Visualization
```python
dap.Steer_Visualization(desired_steer):
#desired_steer is a steer value DPoom should have to reach target location.
```
<br/>
<img src="../easygo/steering_wheel_image.jpg" alt="drawing" width="240"/>
<br/>



This wheel image will show you how should DPoom aim direction to reach the desired point of the given path. __This image should be placed in same dir__.<br/>
dap.py is beta version now. To run this code, call function __easy_test__. This code will be replaced with other code which combine the code with MORP.<br/><br/>

#### __easy_test__
For simple checking whether DPoom move along given path under non_MORP condition.
```python
dap.easy_test(path, velRobot)
#path should be given in 2D array with real numbers, [0] : x, [1] : y coordinates
#velRobot is linear speed [m/s]
```

* This function only exists during beta version.

#### __easy_drive__
```python
dap.easy_drive(goal_x, goal_y, realposition, velRobot)
#goal_x, goal_y is a coordinate of target
#realposition is a coordinate of present position of DPoom. Given by ordered pair
#velRobot is linear speed [m/s]
```

