#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
import time
PI = 3.1415926535897
Magic_value = 1.7375

def main():
    try:
        print('EasyGo Activated')
        #rospy.init_node('robot_easygo', anonymous=False)
        global velocity_publisher
        velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        imu_sub = rospy.Subscriber('imu/yaw', Imu, imu_callback)
        enc_sub = rospy.Subscriber('cmd_vel', Twist, encoder_callback)
    except rospy.ROSInterruptException:
        pass


def printv(text, verbose):
        if verbose==0:
            pass
        elif verbose==1:
            print(text)

def stop(verbose=0):
    #Starts a new node
    #rospy.init_node('robot_mvs', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    vel_msg.linear.x= 0
    velocity_publisher.publish(vel_msg)
    printv('Force STOP', verbose)


def mvRotate(speed, angle, clockwise, verbose=0):
    #Starts a new node
    #rospy.init_node('robot_mvs', anonymous=True)
    #velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    rospy.loginfo("Rotate {0} degree with {1} degree/sec Clockwise = {2}".format(speed, angle, clockwise)  )


    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components


    if angle==-1:
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        velocity_publisher.publish(vel_msg)
        return


    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0


    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
        #print(current_angle)


    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    printv('STOP', verbose)

def mvCurve(x, y, verbose=0):
    #rospy.init_node('robot_mvs', anonymous=True)
    vel_msg = Twist()
    vel_msg.linear.x= x   ###??? x == Robot_speed
    vel_msg.angular.z=y
    velocity_publisher.publish(vel_msg)     ###positive -> clockwise?
def mvStraight(speed, angle, verbose=0):
    #Starts a new node
    #rospy.init_node('robot_mvs', anonymous=True)
    vel_msg = Twist()
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360
    vel_msg.linear.x= angular_speed

    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    if angle == -1:
        printv('inf mode : go straight inf until break', verbose)
        velocity_publisher.publish(vel_msg)
    else:
        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = abs(angular_speed*(t1-t0))
        vel_msg.linear.x= 0
        velocity_publisher.publish(vel_msg)
        printv('STOP', verbose)

def imu_callback(incomming_msg):
    print(incomming_msg)


    return roll, pitch, yaw

def encoder_callback(incomming_msg):
    list_orientation = [incomming_msg.linear.x, incomming_msg.linear.y,
                          incomming_msg.linear.z]

    #list_angular =  [incomming_msg.angular.x, incomming_msg.angular.y,
                           #incomming_msg.angular.z]


    return list_orientation


main()




if __name__ == '__main__':
    try:

        speed = float(input("Input your speed (degrees/sec):"))
        angle = float(input("Type your distance (degrees):"))
        clockwise = input("Clockwise?: ") #True or false
        # Testing our function

        #rotate(speed, angle, clockwise)

        #Verbose = 0 (default) Don;t print status
        #Verbose = 1 Print Everything
        mvStraight(speed, angle, 1)

        '''   Infinity Example ..........break condition DIY
        t0 = rospy.Time.now().to_sec()
        while True:
            mvStraight(velocity_publisher, speed, angle, 1)
            t1 = rospy.Time.now().to_sec()
            if t1-t0 > 10:
                break
        stop()
        '''


    except rospy.ROSInterruptException:
        pass
