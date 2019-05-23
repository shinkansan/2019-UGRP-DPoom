import rospy
rospy.init_node('robot_mvs', anonymous = True)

import easyGo
import easyVector
import math

#######################################################
###########Sample Value for Test!!!!!##################
def get_pathlist():
    return [[0, 0.35, 0.47155, 0.47155, 0.35, 0, -0.35, -0.47155, -0.47155, -0.35, 0, 0, 0, 0],
            [0, 0.60622, 1.29558, 1.99558, 2.68495, 3.29117, 3.89738, 4.58675, 5.28675, 5.97612, 6.58233, 7.28233, 7.98233, 8.68233]]
FlagMORP = False
threshold_boundary = 0.1
Robot_speed = 0.1
#######################################################
#######################################################

def easy_test(path):
    current_position = easyVector.get_poseXY()
    if len(path)==0 or FlagMORP:
        path = get_pathlist()
        i = 0
    if i == len(path)-1:
        print('Goal!')
    else:
        if (path[0][i]-current_position[0])**2 + (path[1][i]-current_position[1])**2 < threshold_boundary**2:
            i = i+1
        easy_drive(path[0][i], path[1][i], current_position)

def easy_drive(goal_x, goal_y, realposition):
    desired_vector = easyVector.main(math.atan2(goal_y-realposition[0], goal_x-realposition[1]))
    easyGo.mvCurve(Robot_speed, desired_vector)

if __name__ == '__main__':
    start_path=[]
    easy_test(start_path)