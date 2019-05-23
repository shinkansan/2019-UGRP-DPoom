#! /usr/bin/env python
# Name : EasyVector
# TODO Get rid of useless code {uncalled}
# HOWTO 
# easyVector.main(speed, desiredAngle)
# First Angle set by IMU
# REMARK : easyVector must be loaded on high hierachy procedure
# Dependencies : imu2angle
# 0517 0444 : add thread, heierachy add
from time import sleep
import signal
import sys
import rospy
import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
import threading
import easyGo
import imu2angle


global current_Angle, last_Angle
global desired_Angle
desired_Angle = 65 # for init working
current_Angle = desired_Angle
Magic_Value = 1.7375 #Value for pin-point turn



def odom_callback(msg):
    # go = Odometry() is not needed
    print "------------------------------------------------"
    print "pose x = " + str(msg.pose.pose.position.x)
    print "pose y = " + str(msg.pose.pose.position.y)
    print "orientacion x = " + str(msg.pose.pose.orientation.x)
    print "orientacion y = " + str(msg.pose.pose.orientation.y)
    rate.sleep()

def imu_callback(msg):
    # allez = Imu()
    print "------------------------------------------------"
    print "veloc angular z = " + str(msg.angular_velocity.z)
    print "veloc angular y = " + str(msg.angular_velocity.y)
    print "aceleracion linear x = " + str(msg.linear_acceleration.x)
    print "aceleracion linear y = " + str(msg.linear_acceleration.y)
    rate.sleep()

def yaw_callback(msg):
    #rospy.init_node('robot_mvs', anonymous=True)
    rate = rospy.Rate(5000)
    global current_Angle, last_Angle
    current_Angle = float(str(msg)[6:12])
    if type(current_Angle) != float:
        #print("")
        last_Angle = current_Angle
    #print(type(current_Angle))
    rate.sleep()

def twist (msg):
    # move = Twist()
    print "velocidad linear x = " + str(move.linear.x)
    print "velocidad angular z = " + str (move.angular.z)
    rate.sleep()
    sub=rospy.Subscriber('cmd_vel', Twist, twist)

def imuThread():
    print('Worker On')
    try:
        imu2angle.init()
    except:
        pass


print('[easyVector INIT]')
#rospy.init_node('robot_mvs', anonymous=True)


#rate = rospy.Rate(5000)

if __name__ == '__main__':
    #================ Publisher & ROS Init =============================
    rospy.init_node('robot_mvs', anonymous=True) # the original name sphero might be the same as other node.
    th = threading.Thread(target=imuThread, name="imuthread")
    th.setDaemon(True)
    th.start()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_imu = rospy.Subscriber('/imu_yaw', Float32, yaw_callback)
     #topic publisher that allows you to move the sphero
    #sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback) # the original name odom might be the same as other function.
    #sub_imu = rospy.Subscriber('/imu_yaw', Float32, yaw_callback)
    
    Kp = 1.7375 / 45
    rate = rospy.Rate(5000)
    while not rospy.is_shutdown():
        cv2.imshow("windows",0)
        k = cv2.waitKey(1)
        if k == ord('k'):
            easyGo.stop()
            print('STOP')
            cv2.waitKey(0)   
        steer = (float(desired_Angle) - float(current_Angle)) * Kp
        if -1*Magic_Value > steer:
            steer = -1*Magic_Value
        elif Magic_Value < steer:
            steer = Magic_Value
        if abs(steer) <= 0.08:
            steer = 0
            print('Reach at Desired Angle')                 
        easyGo.mvCurve(0.5,steer)
        rate.sleep()
    
        # Instead of using rospy.spin(), we should use rate.sleep because we are in a loop

else: #For Dependent Running
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_imu = rospy.Subscriber('/imu_yaw', Float32, yaw_callback)
    th = threading.Thread(target=imuThread, name="imuthread")
    th.setDaemon(True)
    th.start()
    

    pass





def main(desiredAngle, speed=1):
    #rospy.init_node('robot_mvs', anonymous=True)
    rate = rospy.Rate(5000)
    #rate.sleep() #Change if reaction time is slow
    Kp = 1.7375 / 45
    print(current_Angle)
    steer = (float(desiredAngle) - float(current_Angle)) * Kp
    if -1*Magic_Value > steer:
        steer = -1*Magic_Value
    elif Magic_Value < steer:
        steer = Magic_Value
    if abs(steer) <= 0.08:
        steer = 0
        print('Reach at Desired Angle')                 
    return steer
    # TODO I am not sure it's working code, need to be test


