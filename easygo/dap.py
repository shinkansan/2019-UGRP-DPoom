import rospy
rospy.init_node('robot_mvs', anonymous = True)

import easyGo
import easyVector
import math
import matplotlib.pyplot as plt
import cv2

steer = cv2.imread('steering_wheel_image.jpg.png', 0)
rows, cols = steer.shape


#######################################################
###########Sample Value for Test!!!!!##################
def get_pathlist():
    return [[0, 0, 0, 0.5, 1, 1, 1, 0.5, 0, 0, 0, 0.5, 1, 1, 1, 0.5, 0],
            [0, 0.5, 1, 1, 1, 0.5, 0, 0, 0, 0.5, 1, 1, 1, 0.5, 0, 0, 0]]
FlagMORP = False
threshold_boundary = 0.2
Robot_speed = 0.1
x_positions=[]
y_positions=[]
start_position = (0, 0)
#######################################################
#######################################################

def easy_test(path):
    global start_position
    while(True):
        if len(path)==0 or FlagMORP:
            global start_position
            path = get_pathlist()
            i = 1
            while(True):
                start_position = easyVector.get_poseXY()[:]   #### x == forward
                if start_position != (0,0):
                    break

        current_position = (-(easyVector.get_poseXY()[1] - start_position[1]), easyVector.get_poseXY()[0] - start_position[0])   #(x, y)
        print('current_position :', current_position)
        #cv2.waitKey(0)
        #x_positions.append(current_position[0])
        #y_positions.append(current_position[1])
        if i == len(path[0]):
            print('Goal!')
            easyGo.stop()
            break
        else:
            if (path[0][i]-current_position[0])**2 + (path[1][i]-current_position[1])**2 < threshold_boundary**2:
                i = i+1
            print('i:', i)
            easy_drive(path[0][i], path[1][i], current_position)


#############################################################################


def easy_drive(goal_x, goal_y, realposition):  #realposition == current_position(x, y)
    Kp = 1.7375 / 45
    current_angle = easyVector.get_angle()
    print('current_angle :', current_angle)
    if goal_y>=0 :
        desired_angle = -math.atan2(goal_x-realposition[0], goal_y-realposition[1])*180/3.1415926535897
        print('desired_angle :', desired_angle)   ##y_axis == 0 degree
        desired_steer = easyVector.get_steer_Value(desired_angle, Robot_speed)
    else :
        if goal_x>=0:
            desired_angle = -180-math.atan2(goal_x-realposition[0], goal_y-realposition[1])*180/3.1415926535897
            print('desired_angle :', desired_angle)   ##y_axis == 0 degree
            desired_steer = easyVector.get_steer_Value(desired_angle, Robot_speed)
        else:
            desired_angle = 180-math.atan2(goal_x-realposition[0], goal_y-realposition[1])*180/3.1415926535897
            print('desired_angle :', desired_angle)   ##y_axis == 0 degree
            desired_steer = easyVector.get_steer_Value(desired_angle, Robot_speed)

    #######
    easyGo.mvCurve(Robot_speed, desired_steer)
    M = cv2.getRotationMatrix2D((cols / 2, rows / 2), desired_steer*20, 1)
    dst = cv2.warpAffine(steer, M, (cols, rows))
    cv2.imshow("steering wheel", dst)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        easyGo.stop()
        input()
    #easyGo.mvCurve(Robot_speed,0.2)  ##If steer is positive, CCW

if __name__ == '__main__':
    start_path=[]
    easy_test(start_path)

    pipeline.stop()
