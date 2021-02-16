#! /usr/bin/env python

# DRIFT Implement


###### FUNCTIONS #################
## 1. listen odom of SLAM ########
## 2. transform it to grid unit ##
##################################

import rospy
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
from pathplanning import Astar
import numpy as np
from time import sleep
from tf.transformations import euler_from_quaternion
import math
import threading

from easygo import dap
DRIFT_handling_authority = True
flagLocal=False
verbose = 0

class RobotPose:
	def __init__(self):
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

def rotate_origin_only(xy, radians):

    """Only rotate a point around the origin (0, 0)."""
    x, y = xy
    xx = x * math.cos(radians) + y * math.sin(radians)
    yy = -x * math.sin(radians) + y * math.cos(radians)

    return float(xx), float(yy)

def callback(data):
	global pose
	_position = data.pose.pose.position
	pose.x = _position.x
	pose.y = _position.y
	quaternion = (
		data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)
	(_roll, _pitch, pose.rot) = euler_from_quaternion(quaternion) # radians
	if verbose:
		plt.scatter(pose.x, pose.y)
		plt.hold(True)
		plt.pause(0.05)


def listener():
	#rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, callback)
	sleep(2)

def autodrive():

	Astar.pathplanning(start, end, image_path="pathplanning/E5_223.jpg", verbose=0)


def d_main():
	verbose = 0  # if true, show matplot of odometry

	global pose
	global flagLocal
	pose = RobotPose()
	ORIGIN = (83, 10) # mansually calculated. It means index of SLAN map origin on pathplanning binary maze
	# E5-223, (32,25) lobby (83,10)

	#print(pose.convert2grid())

	listener()
	while True:
		if pose.x != 0.0 and pose.y != 0.0:
			flagLocal=False
			break

	flagLocal = True
	print("listener up")

	position_idx = pose.convert2grid()
	print("position in meters: " + str(pose.x) + ", " + str(pose.y))
	print("current_position_idx: " + str(position_idx))
	start = ((int)(position_idx[1] + ORIGIN[0]), (int)(position_idx[0] + ORIGIN[1]))
	print("idx on the map: " + str(start))
	end = (100, 144)  # lobby (55,23) E5_223 (10,37)
	rospy.set_param('/start_x', start[0])
	rospy.set_param('/start_y', start[1])
	final_path = Astar.pathplanning(start, end, image_path="pathplanning/lobby3.jpg", verbose=0) #press Q to quit
	for i,node in enumerate(final_path):
		final_path[i] = [node[0] - start[0], node[1] - start[1]]  # change coordinates and calculate relative path. On the path image, downward(x) and right(y). Because the final coordinates of all module have [robotRight(x), robotStraight(y)].
	#print(final_path)
	final_path_meter = Astar.convert2meter(final_path)
	print("before rotate")
	print(final_path_meter)
	for idx, node in enumerate(final_path_meter):
		final_path_meter[idx] = rotate_origin_only(node, pose.rot)
	print("after rotate")
	print(final_path_meter)
	dap.dap(final_path_meter, velRobot = 0.5, verbose=0)

if __name__ == '__main__':
    rospy.init_node('robot_mvs', anonymous=False)
    d_main()
