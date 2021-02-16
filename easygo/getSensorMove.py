#! /usr/bin/env python
from time import sleep
import signal
import sys
import rospy
import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
import easyGo
global current_Angle, last_Angle
global desired_Angle
desired_Angle = 45
current_Angle = desired_Angle
Magic_Value = 1.7375
def odom_callback(msg):
    # go = Odometry() is not needed
    print "------------------------------------------------"
    print "pose x = " + str(msg.pose.pose.position.x)
    print "pose y = " + str(msg.pose.pose.position.y)
    print "orientacion x = " + str(msg.pose.pose.orientation.x)
    print "orientacion y = " + str(msg.pose.pose.orientation.y)
    rate.sleep()

def imu_callback(msg):
    print "------------------------------------------------"
    print "veloc angular z = " + str(msg.angular_velocity.z)
    print "veloc angular y = " + str(msg.angular_velocity.y)
    print "aceleracion linear x = " + str(msg.linear_acceleration.x)
    print "aceleracion linear y = " + str(msg.linear_acceleration.y)
    rate.sleep()

def yaw_callback(msg):
    global current_Angle, last_Angle
    current_Angle = float(str(msg)[6:12])

    last_Angle = current_Angle
    print(type(current_Angle))
    rate.sleep()


def twist (msg):
    # move = Twist()
    print "velocidad linear x = " + str(move.linear.x)
    print "velocidad angular z = " + str (move.angular.z)
    rate.sleep()
    #sub=rospy.Subscriber('cmd_vel', Twist, twist)

rospy.init_node('robot_mvs', anonymous=True) # the original name sphero might be the same as other node.
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #topic publisher that allows you to move the sphero
#sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback) # the original name odom might be the same as other function.
sub_imu = rospy.Subscriber('/imu_yaw', Float32, yaw_callback)
rate = rospy.Rate(5000)
Kp = 1.7375 / 45

while not rospy.is_shutdown():
    cv2.imshow("img",0)
    k = cv2.waitKey(1)
    if k == ord('k'):
        easyGo.stop()
        cv2.waitKey(0)
        break

    print('cu', current_Angle, 'dr', desired_Angle)

    print(type(current_Angle), type(desired_Angle))
    
    tempInput = (float(desired_Angle) - float(current_Angle)) * Kp
    if -1*Magic_Value > tempInput:
        tempInput = -1*Magic_Value
    elif Magic_Value < tempInput:
        tempInput = Magic_Value
    print(tempInput)
    if abs(tempInput) <= 0.08:
        tempInput = 0
        print('!!!!!!!!!tempInput 0')                 
    easyGo.mvCurve(1,tempInput)
    rate.sleep()
    
    
     # Instead of using rospy.spin(), we should use rate.sleep because we are in a loop
