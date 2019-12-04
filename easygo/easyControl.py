'''
Easy Control Beta
Speed Default 5

    w
a   s   d

e : speed up 0.1
c : speed down 0.2
x : speed default 5
'''
import rospy
rospy.init_node('robot_easygo', anonymous=False)
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
import time
import keyCap
import easyGo
import keyCap


def main():
    speed = 5
    count = 0
    f = keyCap.KeyPoller()
    with f as poller:
        start_time, end_time = 0, 0
        while True:
            getch = poller.poll()
            if getch != None:
                if getch=='w':

                    easyGo.mvStraight(abs(speed), -1)
                    print("[Move Forward] by %f, %d" %(speed, count))
                    keyCap._cls()
                elif getch=="a":
                    easyGo.mvRotate(abs(speed)*4, -1, False)
                    print("[Trun CounterColckwise] by %f, %d" %(speed, count))
                    keyCap._cls()
                elif getch=="s":
                    easyGo.mvStraight(-abs(speed), -1)
                    print("[Move Backward] by %f, %d" %(speed, count))
                    keyCap._cls()
                elif getch=='d':
                    easyGo.mvRotate(abs(speed)*4, -1, True)
                    print("[Trun Colckwise] by %f, %d" %(speed, count))
                    keyCap._cls()
                elif getch=='e':
                    speed+=0.1
                    print('set speed :', speed)
                    keyCap._cls()
                elif getch=="c":
                    speed-=0.2
                    if speed < 0:
                        speed = 0
                    print('set speed :', speed)
                    keyCap._cls()
                elif getch=="x":
                    speed=5
                    print('set speed :', speed)

                elif getch=="q":
                    print('Program terminated')
                    easyGo.stop()
                    return
                else:
                    easyGo.stop()
                count += 1
                start_time = time.time()

            else:
                end_time = time.time()
                if end_time-start_time < 0.1:
                    continue
                easyGo.stop()
                count = 0

main()
