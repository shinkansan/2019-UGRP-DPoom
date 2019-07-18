#! /usr/bin/env python
import rospy
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry

def callback(data):
	print(str(data.pose.pose.position)+"\n")
	_position = data.pose.pose.position
	_x = _position.x
	_y = _position.y
	plt.scatter(_x,_y)
	plt.hold(True)
	plt.pause(0.05)
	

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/rtabmap/odom", Odometry, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
