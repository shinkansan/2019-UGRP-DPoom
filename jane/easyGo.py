#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
PI = 3.1415926535897

def printv(text, verbose):
        if verbose==0:
            pass
        elif verbose==1:
            print(text)

def stop(verbose=0):
    #Starts a new node
    rospy.init_node('robot_goForward', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.linear.x= 0
    velocity_publisher.publish(vel_msg)
    printv('Force STOP', verbose)


def mvRotate(speed, angle, clockwise, verbose=0):
    #Starts a new node
    rospy.init_node('robot_rotate', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    rospy.loginfo("Rotate {0} degree with {1} degree/sec Clockwise = {2}".format(speed, angle, clockwise)  )


    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

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

def mvStraight(pub, speed, angle, verbose=0):
    #Starts a new node
    velocity_publisher = pub
    vel_msg = Twist()
    angular_speed = speed*2*PI/360
    vel_msg.linear.x= angular_speed
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    if angle == -1:
        printv('while mode : go straight inf until break', verbose)
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
    list_orientation = [incomming_msg.orientation.x, incomming_msg.orientation.y,
						   incomming_msg.orientation.z, incomming_msg.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(list_orientation)

    # Showing IMU Data by plot, execute in terminal "rqt_plot"
    pub_imu_roll = rospy.Publisher('IMU_Roll', Float32, queue_size=10)
    pub_imu_pitch = rospy.Publisher('IMU_Pitch', Float32, queue_size=10)
    pub_imu_yaw = rospy.Publisher('IMU_Yaw', Float32, queue_size=10)


    return roll, pitch, yaw

def encoder_callback(incomming_msg):
    list_orientation = [incomming_msg.linear.x, incomming_msg.linear.y,
						  incomming_msg.linear.z]
    #list_angular =  [incomming_msg.angular.x, incomming_msg.angular.y,
    # #incomming_msg.angular.z]
    return list_orientation

if __name__ == '__main__':
    try:
        rospy.init_node('robot_easygo', anonymous=True)
        velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        imu_sub = rospy.Subscriber('imu', Imu, imu_callback)
        imu_sub = rospy.Subscriber('cmd_vel', Twist, encoder_callback)
        speed = float(input("Input your speed (degrees/sec):"))
        angle = float(input("Type your distance (degrees):"))
        clockwise = input("Clockwise?: ") #True or false
        # Testing our function
        #rotate(speed, angle, clockwise)
        #Verbose = 0 (default) Don;t print status
        #Verbose = 1 Print Everything

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
