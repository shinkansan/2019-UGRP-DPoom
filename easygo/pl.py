import rospy
rospy.init_node('robot_mvs', anonymous = True)

import matplotlib.pyplot as plt
import easyGo
import easyVector
import numpy as np
import math

fig, ax = plt.subplots()
x = [0, 0, 0, 0.5, 1, 1, 1, 0.5, 0, 0, 0, 0.5, 1, 1, 1, 0.5, 0]
y = [0, 0.5, 1, 1, 1, 0.5, 0, 0, 0, 0.5, 1, 1, 1, 0.5, 0, 0, 0]
threshold_boundary = 0.2   #0.14

plt.scatter(x, y, color = 'b')
plt.hold(True)
plt.xlim(-3, 3)
plt.ylim(-3, 10)
plt.hold(True)

while(True):
    start_position = easyVector.get_poseXY()[:]   #### x == forward
    if start_position != (0,0):
        break
i = 1

print('start_position :',start_position)

while(True):
    print('get_poseXY :', easyVector.get_poseXY())
    cp_x = -(easyVector.get_poseXY()[1]-start_position[1])
    cp_y = easyVector.get_poseXY()[0]-start_position[0]
    print(cp_x, cp_y)
    plt.scatter(cp_x, cp_y, c='r')
    plt.hold(True)
    current_angle = easyVector.get_angle()
    #V=np.array([[cp_x+math.sin(current_angle), cp_y+math.cos(current_angle)],])
    #present robot's direction & desired direction
    ############################################
    #to get desired direction, pl.py should work with dap.py (cause of goal point)
    ############################################
    #plt.quiver((cp_x, cp_y), )
    # present robot's direction
    #plt.arrow(cp_x, cp_y, math.cos((current_angle+90)/180*math.pi), math.sin((current_angle+90)/180*math.pi), head_width = 0.03, head_length = 0.2, color = 'r')

    # desired direction
    if i == len(x)-1:
        break
    else:
        if (x[i]-cp_x)**2 + (y[i]-cp_y)**2 < threshold_boundary**2:
            i = i+1
    desired_angle = math.atan2(x[i]-cp_x, y[i]-cp_y)*180/3.14159
    #plt.arrow(cp_x, cp_y, math.sin(desired_angle), math.cos(desired_angle), color = 'b')
    plt.pause(0.05)
plt.show()
