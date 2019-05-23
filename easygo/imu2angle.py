#!/usr/bin/env python

import rospy as _rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rad2degrees = 180.0/math.pi
degrees2rad = math.pi / 180.0

class Imu2Angles:
    def __init__(self):

        self.rate = _rospy.get_param('~rate', 2000.0)
        self.imu_name = _rospy.get_param('~imu_name', 'imu')
        self.topic_name = _rospy.get_param('~topic_name', 'imu')

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.pub_imu_roll_msg = Float32()
        self.pub_imu_pitch_msg = Float32()
        self.pub_imu_yaw_msg = Float32()

        self.pub_imu_roll = _rospy.Publisher('/' + self.imu_name +'/roll', Float32, queue_size=1)
        self.pub_imu_pitch = _rospy.Publisher('/' + self.imu_name +'/pitch', Float32, queue_size=1)
        self.pub_imu_yaw = _rospy.Publisher('/imu_yaw', Float32, queue_size=1)
        
        self.sub = _rospy.Subscriber(self.topic_name, Imu, self.process_imu_message, queue_size=1)

        rate = _rospy.Rate(self.rate)
	_rospy.spin()
        while not _rospy.is_shutdown():

            rate.sleep()

    def process_imu_message(self, imuMsg):
        quaternion = (
            imuMsg.orientation.x,
            imuMsg.orientation.y,
            imuMsg.orientation.z,
            imuMsg.orientation.w)

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
    _rospy.init_node('robot_mvs', anonymous=True)

    try:
        obj = Imu2Angles()
    except:
        pass

def init():

    try:
        obj = Imu2Angles()
    except:
        pass
