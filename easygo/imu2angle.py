#!/usr/bin/env python

import rospy as _rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

rad2degrees = 180.0/math.pi
degrees2rad = math.pi / 180.0


class Imu2Angles:
    def __init__(self):
        print('fuck2')
        self.rate = _rospy.get_param('~rate', 2000.0)
        print('fuck2')
        #self.imu_name = _rospy.get_param('~imu_name', 'imu')

        self.imu_name = 'imu'
        self.topic_name = _rospy.get_param('~topic_name', 'imu')

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0


        #self.pub_imu_roll_msg = Float32()
        #self.pub_imu_pitch_msg = Float32()
        #self.pub_imu_yaw_msg = Float32()

        print('hello3')
        self.pub_imu_roll = _rospy.Publisher('/' + self.imu_name +'/roll', Float32, queue_size=1)
        print('hello3')
        self.pub_imu_pitch = _rospy.Publisher('/' + self.imu_name +'/pitch', Float32, queue_size=1)

        self.pub_imu_yaw = _rospy.Publisher('/imu_yaw', Float32, queue_size=1)
        print('hello2')
        self.sub = _rospy.Subscriber('/odom', Odometry, self.process_imu_message, queue_size=1)

        rate = _rospy.Rate(self.rate)
        print('hello')
        #_rospy.spin()
        #print('hello2')
        #while not _rospy.is_shutdown():
        #    print('hello')
        #    rate.sleep()
        _rospy.spin()
        rate = _rospy.Rate(5000)
        print('thread re3')
        while not _rospy.is_shutdown():
            rate.sleep()
        print('thread re2')

    def process_imu_message(self, imuMsg):
        quaternion = (
            imuMsg.pose.pose.orientation.x,
            imuMsg.pose.pose.orientation.y,
            imuMsg.pose.pose.orientation.z,
            imuMsg.pose.pose.orientation.w)

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)

        self.roll = self.roll * rad2degrees
        self.pitch = self.pitch * rad2degrees
        self.yaw = self.yaw * rad2degrees

        self.publish_angles()

    def publish_angles(self):
        self.pub_imu_roll_msg = Float32()
        self.pub_imu_roll_msg.data = self.roll
        self.pub_imu_roll.publish(self.pub_imu_roll_msg)

        self.pub_imu_pitch_msg = Float32()
        self.pub_imu_pitch_msg.data = self.pitch
        self.pub_imu_pitch.publish(self.pub_imu_pitch_msg)

        self.pub_imu_yaw_msg = Float32()
        self.pub_imu_yaw_msg.data = self.yaw
        self.pub_imu_yaw.publish(self.pub_imu_yaw_msg)


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    print("asdf")
    _rospy.init_node('robot_mvs', anonymous=True)
    print("asdf")
    #try:
    obj = Imu2Angles()
    #except:
        #pass

def init():
    #_rospy.init_node('robot_mvs', anonymous=True)
    #try:
    obj = Imu2Angles()
    #except Exception as rex:
        #print(rex);pass
