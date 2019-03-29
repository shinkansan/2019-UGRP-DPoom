# ROS easyGo and ros docs
===================

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

# How to catkin_make
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

# easyGo

Source: [./easygo/easyGo.py](./easygo/easygo.py)


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
easyGo.mvRotate(pub, speed, angle, clockwise, verbose=0)
'''Rotate {0} degree with {1} degree/sec Clockwise = {2}'''
```
pub is for Publisher, Example code is below


### Forward/Backward
```bash
easyGo.mvStraight(pub, speed, angle, verbose=0)
#Stop All Vector (x,y,z)
```
angle = -1 is for inf go, the example code is below


### function example
```bash
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
