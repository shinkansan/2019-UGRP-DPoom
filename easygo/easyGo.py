#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
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
        
    


if __name__ == '__main__':
    try:
        rospy.init_node('robot_goForward', anonymous=True)
        velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
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
