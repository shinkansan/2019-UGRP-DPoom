import rospy
rospy.init_node('robot_mvs', anonymous = True)
import easyGo
import easyVector
import math
import matplotlib.pyplot as plt
import cv2
stopper_sender = True
steer = cv2.imread('steering_wheel_image.jpg.png', 0)
rows, cols = steer.shape

#######################################################
###########Sample Value for Test!!!!!##################
FlagMORP = False
threshold_boundary = 0.2
start_position = (0, 0)
PI = 3.1415926535897
#######################################################
#######################################################


easyGo.stopper = True


def Steer_Visualization(desired_steer):     #steer visualization
    M = cv2.getRotationMatrix2D((cols / 2, rows / 2), desired_steer*20, 1)
    dst = cv2.warpAffine(steer, M, (cols, rows))
    cv2.imshow("steering wheel", dst)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        easyGo.stop()
        input()

def dap(path, velRobot=0.1, verbose=0):     # path is 2D array (x, y)
    global start_position
    i=0
    while(True):
        #print('thread re1fuck')
        if i==0 or FlagMORP:     # when just running code or we should search new path(by MORP algorithm)
            while(True):     # at first time, get_poseXY() == (0, 0)
                start_position = easyVector.get_poseXY()[:]   # x axis == forward
                print(start_position, easyVector.get_angle())
                if start_position != (0.0,0.0):
                    break
            if verbose: # plot current robot position
				fig, ax = plt.subplots()
				threshold_boundary = 0.2   #0.14
				_x = [item[0] for item in path]
				_y = [item[1] for item in path]
				plt.scatter(_x, _y, color = 'b')
				plt.hold(True)
				plt.xlim(-1, 1.3)
				plt.ylim(-2, 2)
				plt.hold(True)
        current_position = (-(easyVector.get_poseXY()[1] - start_position[1]),
                                easyVector.get_poseXY()[0] - start_position[0])
        #(x, y) when DPoom head to right, x decreases (negatively increases)
        # current_position and DAP's coordinates : straight(y), right(x)
        #print('current_position :', current_position)
        if (path[i][0]-current_position[0])**2 + (path[i][1]-current_position[1])**2 < threshold_boundary**2:
            i = i+1
            if i == len(path):
                print('Goal!')
                easyGo.stop()
                if verbose:
                    plt.close()
                break
        print('i:', i)
        if cv2.waitKey(1) == ord('k'):
            easyGo.stop()
            exit()
            break
        easy_drive(path[i][0], path[i][1], current_position, velRobot)
        if verbose:
            plt.scatter(current_position[0], current_position[1], c='r')
            plt.hold(True)
            plt.pause(0.05)
            #plt.show()

#############################################################################

def easy_drive(goal_x, goal_y, realposition, velRobot):  #realposition == current_position(x, y)
    Kp = 1.7375 / 10
    current_angle = easyVector.get_angle()    ##y_axis == 0 degree, CW -> 0 ~ +180 degree
    print('current_angle :', current_angle)
    if goal_y-realposition[1]>=0 :   # pi/2 <= desired_angle <= pi/2
        desired_angle = -math.atan2(goal_x-realposition[0],
                                            goal_y-realposition[1])*180/PI
        #print('desired_angle :', desired_angle)
        desired_steer = easyVector.get_steer_Value(desired_angle, velRobot)
    else :   #abs(desired_angle) >= pi/2
        desired_angle = -math.atan2(goal_x-realposition[0],
                                            goal_y-realposition[1])*180/PI
        #print('desired_angle :', desired_angle)
        desired_steer = easyVector.get_steer_Value(desired_angle, velRobot)
    print('desired_angle :', desired_angle)
    easyGo.stopper = stopper_sender
    easyGo.mvCurve(velRobot, desired_steer)

    ###############Steer visualization###############
    Steer_Visualization(desired_steer)
    ################################################

if __name__ == '__main__':
    #S
    '''
    test_path=[[0, 0.25, 0.37155, 0.37155, 0.25, 0, -0.25, -0.37155, -0.37155, -0.25, 0, 0, 0, 0],
            [0, 0.60622, 1.29558, 1.99558, 2.68495, 3.29117, 3.89738, 4.58675, 5.28675, 5.97612, 6.58233, 7.28233, 7.98233, 8.68233]]
    '''
    #star
    '''
    test_path = [[0, -0.3333, -0.5, -0.7, -1,
    -0.5, 0, 0.5, 0.9, 1.2,
    0.8, 0.3333, -0.3333, -0.8, -1.2,
    -0.8, -0.5,  0, 0.5, 1,
    0.7, 0.5, 0.3333, 0],
    [0, -0.6667, -1.0909, -1.5, -2,
    -1.6535, -1.3030, -1.0909, -0.9, -0.6757,
     -0.6667, -0.6667, -0.6667, -0.6667, -0.6667,
     -0.9, -1.0909, -1.3030, -1.6535, -2,
     -1.5, -1.0909, -0.6667, 0]]
     '''
    #rectangle
    '''
    test_path=[[0, 0, 0, 0.5, 1, 1, 1, 0.5, 0, 0, 0, 0.5, 1, 1, 1, 0.5, 0],
    [0, 0.5, 1, 1, 1, 0.5, 0, 0, 0, 0.5, 1, 1, 1, 0.5, 0, 0, 0]]
    '''
    #randomly
    '''
    test_path=[[0, 0.5, 1, 0.7, 0, -0.2, -0.5, 0.5],
    [0, 1.2, 0.5, 0.8, 0.6, 1.3, 0.8, 1.5]]
    '''

    test_path = [[0, 0], [0.5, 0], [1, 0], [1, 0.5], [1, 1], [0.5, 1], [0, 1], [0, 0.5], [0, 0]]
    print('test')
    # velRobot = 0.1
    dap(test_path, verbose=1)
