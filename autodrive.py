#! /usr/bin/env python

###### FUNCTIONS #################
## 1. listen odom of SLAM ########
## 2. transform it to grid unit ##
##################################

import rospy
from matplotlib import pyplot as plt
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from pathplanning.Astar import pathplanning
import numpy as np
from time import sleep

class RobotPose:
	def __init__(self):
		#_data = rospy.wait_for_message("/rtabmap/localization_pose", PoseWithCovarianceStamped)
		#_position = _data.pose.pose.position
		self.x = 0.0
		self.y = 0.0
		self.rot = 0.0

	def setPose(self, x, y, rot):
		self.x = x
		self.y = y
		self.rot = rot
		return (x, y, rot)

	def convert2grid(self, scale=0.2):
		x, y = round(self.x/scale), - round(self.y/scale) # give a minus to 'y', because y-axis towars left side of robot
		return (x, y)

def callback(data):
	#print(str(data.pose.pose.position)+"\n")
	_position = data.pose.pose.position
	pose.x = _position.x
	pose.y = _position.y
	if verbose:
		plt.scatter(pose.x, pose.y)
		plt.hold(True)
		plt.pause(0.05)


def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, callback)
	sleep(2)
	#rospy.spin()

def autodrive():

	pathplanning(start, end, image_path="pathplanning/E5_223.jpg", verbose=0)

if __name__ == '__main__':
	verbose = 0  # if true, show matplot of odometry
	
	#_data = rospy.wait_for_message("/rtabmap/localization_pose", PoseWithCovarianceStamped, 15.0)
	#rospy.spin()
	#print("init pose")
	pose = RobotPose()
	ORIGIN = (30, 20) # manually calculated. It means index of SLAN map origin on pathplanning binary maze

	#print(pose.convert2grid())
	
	listener()
	print("listener up")
	
	position_idx = pose.convert2grid()
	print("position in meters: " + str(pose.x) + ", " + str(pose.y))
	print("current_position_idx: " + str(position_idx))
	start = (position_idx[1] + ORIGIN[0], position_idx[0] + ORIGIN[1])
	print("idx on the map: " + str(start))
	end = (10, 37)

	pathplanning(start, end, image_path="pathplanning/E5_223.jpg", verbose=1)
	
	
