import rospy
rospy.init_node('robot_mvs', anonymous = True)

import matplotlib.pyplot as plt
import easyGo
import easyVector
import numpy as np
import math

fig, ax = plt.subplots()
#S

'''
x = [0, 0.25, 0.37155, 0.37155, 0.25, 0, -0.25, -0.37155, -0.37155, -0.25, 0, 0, 0, 0]
y = [0, 0.60622, 1.29558, 1.99558, 2.68495, 3.29117, 3.89738, 4.58675, 5.28675, 5.97612, 6.58233, 7.28233, 7.98233, 8.68233]
'''
#star
'''
x = [0, -0.3333, -0.5, -0.7, -1,
-0.5, 0, 0.5, 0.9, 1.2,
0.8, 0.3333, -0.3333, -0.8, -1.2,
-0.8, -0.5,  0, 0.5, 1,
0.7, 0.5, 0.3333, 0]
y = [0, -0.6667, -1.0909, -1.5, -2,
-1.6535, -1.3030, -1.0909, -0.9, -0.6757,
 -0.6667, -0.6667, -0.6667, -0.6667, -0.6667,
 -0.9, -1.0909, -1.3030, -1.6535, -2,
 -1.5, -1.0909, -0.6667, 0]
'''
#rectangle
'''
x=[0, 0, 0, 0.5, 1, 1, 1, 0.5, 0, 0, 0, 0.5, 1, 1, 1, 0.5, 0]
y=[0, 0.5, 1, 1, 1, 0.5, 0, 0, 0, 0.5, 1, 1, 1, 0.5, 0, 0, 0]
'''
#randomly

x=[0, 0.5, 1, 0.7, 0, -0.2, -0.5, 0.5]
y=[0, 1.2, 0.5, 0.8, 0.6, 1.3, 0.8, 1.5]


threshold_boundary = 0.2   #0.14

plt.scatter(x, y, color = 'b')
plt.hold(True)
plt.xlim(-1, 1.3)
plt.ylim(0, 2)
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
