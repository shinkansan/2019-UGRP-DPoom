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



def convert2grid(position, scale=0.2):
	position = [round(position[0]/scale), round(position[1]/scale)]
	return position

def callback(data):
	print(str(data.pose.pose.position)+"\n")
	_position = data.pose.pose.position
	_x = _position.x
	_y = _position.y
	plt.scatter(_x,_y)
	plt.hold(True)
	plt.pause(0.05)


def listener():

	map_origin_index =
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, callback)
	rospy.spin()

if __name__ == '__main__':
	print(convert2grid([0.3, 0.09]))
	listener()
