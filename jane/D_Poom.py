import os
import time
import keyboard
import msvcrt
import math
from soscon.env import Env
from soscon.status import Status
from soscon.data.observation import Observation
import soscon.status
import cv2
import numpy as np
import matplotlib.pyplot as plt

'''
wasd는 누르고 있으면 작동:
w: 전진
s: 후진
a: 좌회전
d: 우회전

[,]은 정확한 90도 턴을 위함:
[: 왼쪽으로 90도 턴
]: 오른쪽으로 90도 턴
'''

'''
1 means 1 degree, and it is pi/36 cm
The bounding box of door, y value is small than 10cm
'''

DRIVE_SPEED = 100
ANGULAR_SPEED = 60


class RobotController():
    LIDAR_DATA_SIZE = 360
    IR_DATA_SIZE = 5
    ### Tracing Variables on Class
    initStatus = False
    traceMapInit = False
    gridH = 0
    gridW = 0
    MapH = 0
    MapW = 0
    GlobalMap = 0
    PathSaveVal = []
    PathSaveCn = []
    diameter = 0
    index_path = 1

    ###

    def __init__(self):
        self._env = Env()

        self._total_dx = 0
        self._total_dy = 0
        self._prev_obs = None

        self._env.on_observation = self.on_observation
        self.stop()

    def _cls(self):
        os.system("cls" if os.name == "nt" else "clear")

    def on_observation(self, obs: Observation, status: Status):
        if status is Status.OK:
            self._prev_obs = obs
            self._total_dx += obs.delta.x
            self._total_dy += obs.delta.y

    def print_current_state(self):
        if self._prev_obs is None:
            print("[ERROR] No measured not yet")
            return

        if len(self._prev_obs.lidar) != self.LIDAR_DATA_SIZE:
            print("[ERROR] LiDAR was not measured")
            return

        if len(self._prev_obs.ir) != self.IR_DATA_SIZE:
            print("[ERROR] IR was not measured")
            return

        self._cls()

        print("[Compass] %.1f deg" % self._prev_obs.compass)
        print("[Encoder] Left: %d, Right: %d" % (self._prev_obs.encoder.left, self._prev_obs.encoder.right))
        print("[IR] %.1f %.1f %.1f %.1f %.1f cm" % (
            self._prev_obs.ir[0], self._prev_obs.ir[1], self._prev_obs.ir[2], self._prev_obs.ir[3],
            self._prev_obs.ir[4]))
        print("[LiDAR] %.1f %.1f %.1f cm" % (
            self._prev_obs.lidar[270], self._prev_obs.lidar[0], self._prev_obs.lidar[90]))
        print("[Delta] X: %.1f, Y: %.1f" % (self._total_dx, self._total_dy))

    def stop(self):
        print("[STOP]")
        self._env.control_robot(0.0, 0.0)
        self._total_dx = 0
        self._total_dy = 0

    def move(self, linear_velocity: float, angular_velocity: float):
        self._env.control_robot(linear_velocity, angular_velocity)

    def check_front_ir(self) -> float:
        if self._prev_obs is None:
            raise RuntimeError("[ERROR] No measured not yet")

        if len(self._prev_obs.ir) != self.IR_DATA_SIZE:
            raise RuntimeError("[ERROR] IR was not measured")

        return min(self._prev_obs.ir[1:4])

    def check_front_lidar(self) -> float:
        if self._prev_obs is None:
            raise RuntimeError("[ERROR] No measured not yet")

        if len(self._prev_obs.lidar) != self.LIDAR_DATA_SIZE:
            raise RuntimeError("[ERROR] LiDAR was not measured")

        left_min = min(self._prev_obs.lidar[345:360])
        right_min = min(self._prev_obs.lidar[0:16])
        return min([left_min, right_min])

    def move_forward(self, distance_cm: float):
        self.stop()
        print("[MOVE_FORWARD] Distance(cm): %.1f" % (distance_cm))

        while True:
            time.sleep(0.05)

            target_velocity = distance_cm - self._total_dx
            if target_velocity <= 5:
                target_velocity = 5
            elif target_velocity > DRIVE_SPEED:
                target_velocity = DRIVE_SPEED

            robot.move(target_velocity, 0)
            if self._total_dx >= distance_cm:
                self.stop()
                break

    def rotate_counter_cw2(self, target_angle_deg: float):
        self.stop()

        print("[ROTATE_CW] Angle(deg): %.1f" % (target_angle_deg))
        prev_compass = self._prev_obs.compass
        while True:
            robot.print_current_state()
            time.sleep(0.05)
            print("ROTATING BY STATE !!!!!!!!!!!!!" + str(state01))
            current_compass = self._prev_obs.compass
            delta_deg = prev_compass - current_compass
            if delta_deg < 0:
                delta_deg = 360 + prev_compass - current_compass

            target_velocity = abs(abs(target_angle_deg) - abs(delta_deg)) * 1.3
            if target_velocity <= 2:
                target_velocity = 2
            elif target_velocity <= 3:
                target_velocity = 3
            elif target_velocity > 30:
                target_velocity = 30
            print(target_velocity)
            print(delta_deg)
            print(target_angle_deg)
            if abs(delta_deg) >= abs(target_angle_deg):
                self.stop()
                break
            else:
                self.move(0, target_velocity)
        time.sleep(0.2)

    def rotate_cw2(self, target_angle_deg: float):
        global state01, boolean03, target_angle_global
        self.stop()
        print("[ROTATE_CW] Angle(deg): %.1f" % (target_angle_deg))
        prev_compass = self._prev_obs.compass
        desired_deg = prev_compass + target_angle_deg
        if desired_deg < 0:
            desired_deg = desired_deg + 360
        if desired_deg > 360:
            desired_deg = desired_deg - 360
        while True:
            robot.print_current_state()
            time.sleep(0.05)
            print("ROTATING BY STATE !!!!!!!!!!!!!" + str(state01))
            current_compass = self._prev_obs.compass
            delta_deg = current_compass - prev_compass
            if delta_deg < 0:
                delta_deg = 360 + current_compass - prev_compass

            target_velocity = abs(abs(desired_deg) - abs(current_compass)) * 1.3
            if target_velocity <= 2:
                target_velocity = 2
            elif target_velocity <= 3:
                target_velocity = 3
            elif target_velocity > 30:
                target_velocity = 30
            print(target_velocity)
            if abs(delta_deg) >= abs(target_angle_deg):
                self.stop()
                break
            else:
                self.move(0, -1 * target_velocity)

        time.sleep(0.2)

    def rotate_cw(self, input01: float):
        global flag01, target_angle, initial_compass, flag02, desired_deg, state01, flag_lost_rotate, init_encoder
        if flag01 == True:  # initializing
            self.stop()
            target_angle = input01
            initial_compass = self._prev_obs.compass
            desired_deg = initial_compass + target_angle
            if desired_deg < 0:
                desired_deg = desired_deg + 360
            if desired_deg > 360:
                desired_deg = desired_deg - 360
            if desired_deg < 3:
                desired_deg = 360
            flag01 = False
            flag02 = True
        else:

            robot.print_current_state()
            time.sleep(0.05)
            print("ROTATING BY STATE !!!!!!!!!!!!!" + str(state01))
            current_compass = self._prev_obs.compass
            delta_deg = current_compass - initial_compass
            if abs(delta_deg) < 3:
                delta_deg = 0
            if delta_deg < 0:
                delta_deg = 360 + current_compass - initial_compass

            print(delta_deg)
            print(target_angle)
            target_velocity = abs(abs(desired_deg) - abs(current_compass)) * 1.3
            if target_velocity > 300:
                target_velocity = 30
            elif target_velocity <= 5:  # let this number be 2 to accurate turn
                target_velocity = 5
            elif target_velocity <= 15:  # let this number be 3 to accurate turn
                target_velocity = 15
            elif target_velocity > ANGULAR_SPEED:
                target_velocity = ANGULAR_SPEED
            if abs(delta_deg) +0 >= abs(target_angle):
                self.stop()
                time.sleep(0.2)
                flag01 = True
                flag02 = False
                flag_lost_rotate = False
                init_encoder = self._prev_obs.encoder.left
                return True
            else:
                self.move(0, -1 * target_velocity)
                init_encoder = self._prev_obs.encoder.left
                return False

    def rotate_counter_cw(self, input01: float):
        global flag01, target_angle, initial_compass, flag03, desired_deg, state01, flag_lost_rotate, init_encoder
        if flag01 == True:  # initializing
            self.stop()
            target_angle = input01
            initial_compass = self._prev_obs.compass
            desired_deg = initial_compass + target_angle
            flag01 = False
            flag03 = True
        else:
            robot.print_current_state()
            time.sleep(0.05)
            print("ROTATING BY STATE !!!!!!!!!!!!!" + str(state01))
            current_compass = self._prev_obs.compass
            delta_deg = -current_compass + initial_compass
            if abs(delta_deg) < 0.1:
                delta_deg = 0
            if delta_deg < 0:
                delta_deg = 360 - current_compass + initial_compass

            target_velocity = abs(abs(target_angle) - abs(delta_deg)) * 1.3
            if target_velocity <= 5:  # let this number be 2 to accurate turn
                target_velocity = 5
            elif target_velocity <= 15:  # let this number be 3 to accurate turn
                target_velocity = 15
            elif target_velocity > ANGULAR_SPEED:
                target_velocity = ANGULAR_SPEED
            if abs(delta_deg) + 0 >= abs(target_angle):
                self.stop()
                time.sleep(0.2)
                flag01 = True
                flag03 = False
                flag_lost_rotate = False
                init_encoder = self._prev_obs.encoder.left
                return True
            else:
                self.move(0, target_velocity)
                init_encoder = self._prev_obs.encoder.left
                return False

    def easy_control(self):
        print("[EASY CONTROL]")
        global boolean01, flag02, flag03, flag01, flag04, flag_boundingbox_check, flag_bb_cm, flag_bb_cm2, flag05, flag06, flag08

        if keyboard.is_pressed('p'):
            robot.stop()
            time.sleep(1)
            boolean01 = True
            flag02 = False
            flag03 = False
            flag04 = False
            flag01 = True
            flag05 = False
            flag06 = False
            flag08 = False
            flag_bb_cm = False
            flag_bb_cm2 = False
            flag_boundingbox_check = False

        # parameter in those, they have no meaning
        elif flag03 == True:
            robot.rotate_counter_cw(90)
        elif flag04 == True:
            robot.move_forward_by_cm(1)
        elif flag02 == True:
            robot.rotate_cw(90)
        elif flag_boundingbox_check == True:
            if keyboard.is_pressed('c'):
                flag_boundingbox_check = False
            return 0

        elif keyboard.is_pressed('w'):   ## move forward
            if self._prev_obs.lidar[0] <= 41:
                robot.stop()
            else:
                robot.move(60, 0)
        elif keyboard.is_pressed('s'):   ## move backward
            robot.move(-60, 0)
        elif keyboard.is_pressed('a'):  ## move left
            robot.move(0, 60)
        elif keyboard.is_pressed('d'):   ## move right
            robot.move(0, -60)
        elif keyboard.is_pressed('['):
            robot.rotate_counter_cw(90)
        elif keyboard.is_pressed(']'):
            robot.rotate_cw(90)
        elif keyboard.is_pressed(' '):
            robot.stop()
        elif keyboard.is_pressed('o'):
            robot.wall_trace(DRIVE_SPEED)
        elif keyboard.is_pressed('i'):
            robot.move_forward_by_cm(55)
        elif keyboard.is_pressed('c'):
            for i in self._prev_obs.boundingbox:
                print(i)
            flag_boundingbox_check = True
            time.sleep(30)
        elif keyboard.is_pressed('u'):
            robot.compass_trace(DRIVE_SPEED)
        elif keyboard.is_pressed('b'):
            print(robot.bb_scan())
        elif keyboard.is_pressed('m'):
            robot.is_wall_obs()
        elif keyboard.is_pressed('n'):
            robot.auto_scan_for_cm(68)
        elif keyboard.is_pressed('j'):
            robot.move(20, 0)

        else:
            robot.stop()

    def Hansel_and_Gretel(self):
        map = self.GlobalMap
        # print(self.PathSave)
        # region_0 = (self.gridH + self.diameter, self.gridW)
        if (self.TPO(self._prev_obs.compass) == 0):
            region_270 = (self.gridH, self.gridW - self.diameter)

            region_front = (self.gridH + self.diameter, self.gridW)
            if ([region_270[0], region_270[1]] in self.PathSaveVal):
                left_index = self.PathSaveCn[self.PathSaveVal.index([region_270[0], region_270[1]])]

            else:
                left_index = 0
            if ([region_front[0], region_front[1]] in self.PathSaveVal):
                front_index = self.PathSaveCn[self.PathSaveVal.index([region_front[0], region_front[1]])]
            else:
                front_index = 0
            Sboundary = self.cnt2bnd(self.gridH, self.gridW - self.diameter, self.diameter)
            cv2.rectangle(map, (Sboundary[0]), (Sboundary[1]), (255, 175, 175), -1)
        elif (self.TPO(self._prev_obs.compass) == 180):
            region_270 = (self.gridH, self.gridW + self.diameter)
            region_front = (self.gridH - self.diameter, self.gridW)
            if ([region_270[0], region_270[1]] in self.PathSaveVal):
                left_index = self.PathSaveCn[self.PathSaveVal.index([region_270[0], region_270[1]])]
            else:
                left_index = 0
            if ([region_front[0], region_front[1]] in self.PathSaveVal):
                front_index = self.PathSaveCn[self.PathSaveVal.index([region_front[0], region_front[1]])]
            else:
                front_index = 0
            Sboundary = self.cnt2bnd(self.gridH, self.gridW + self.diameter, self.diameter)
            cv2.rectangle(map, (Sboundary[0]), (Sboundary[1]), (255, 175, 175), -1)
        elif (self.TPO(self._prev_obs.compass) == 90):
            region_270 = (self.gridH + self.diameter, self.gridH)
            region_front = (self.gridH, self.gridW + self.diameter)
            if ([region_270[0], region_270[1]] in self.PathSaveVal):
                left_index = self.PathSaveCn[self.PathSaveVal.index([region_270[0], region_270[1]])]
            else:
                left_index = 0
            if ([region_front[0], region_front[1]] in self.PathSaveVal):
                front_index = self.PathSaveCn[self.PathSaveVal.index([region_front[0], region_front[1]])]
            else:
                front_index = 0
            Sboundary = self.cnt2bnd(self.gridH, self.gridW + self.diameter, self.diameter)
            cv2.rectangle(map, (Sboundary[0]), (Sboundary[1]), (255, 175, 175), -1)
        elif (self.TPO(self._prev_obs.compass) == 270):
            region_270 = (self.gridH - self.diameter, self.gridH)
            region_front = (self.gridH, self.gridW + self.diameter)
            if ([region_270[0], region_270[1]] in self.PathSaveVal):
                left_index = self.PathSaveCn[self.PathSaveVal.index([region_270[0], region_270[1]])]
            else:
                left_index = 0
            if ([region_front[0], region_front[1]] in self.PathSaveVal):
                front_index = self.PathSaveCn[self.PathSaveVal.index([region_front[0], region_front[1]])]
            else:
                front_index = 0
            Sboundary = self.cnt2bnd(self.gridH, self.gridW + self.diameter, self.diameter)
            cv2.rectangle(map, (Sboundary[0]), (Sboundary[1]), (255, 175, 175), -1)
        else:
            print("ERROR!!!!! Not Adequate Compass : Unknown ERROR", self.TPO(self._prev_obs.compass))
            # time.sleep(3)
            return True
        # print(tuple(region_270))
        print(left_index >= 1 or front_index >= 2)
        Boolean = left_index >= 1 or front_index >= 2
        # print(Boolean)
        # time.sleep(3)
        if (Boolean):
            print("Hansel and Gretel Find a snack!!!!!!!! now, they can go home")
            print("left snack qty : ", left_index, " front snack qty : ", front_index)
            # time.sleep(5)
            return True
        else:
            print("Since there is no snack around the left, so they decide to explore that direction")
            print("left snack qty : ", left_index, " front snack qty : ", front_index)
            return False

    def TraceMapping(self, **size):
        # How to use TraceMapping(diameter=34, ...etc)  width and hpoint,  wpoint for manual height, width - orientation
        # width, height, diameter is already set
        if size.get('showmode', False) == False:
            self.diameter = int(size.get('diameter', 10))
            if (self.traceMapInit == False):
                self.traceMapInit = True
                print('Initailizing TraceMap')
                # time.sleep(3)
                self.MapH = size.get('height', 500)
                self.MapW = size.get('width', 500)
                new_image = np.ndarray((3, self.MapH, self.MapW), dtype=int)
                # Converted the datatype to np.uint8
                new_image = new_image.astype(np.uint8)
                # Separated the channels in my new image
                new_image_red, new_image_green, new_image_blue = new_image
                # Stacked the channels
                map = np.dstack([new_image_red, new_image_green, new_image_blue])
                # Displayed the image

                map = cv2.cvtColor(map, cv2.COLOR_RGB2BGR)
                hpoint = int(size.get('hpoint', self.MapH / 2))
                wpoint = int(size.get('wpoint', self.MapW / 2))
                self.PathSaveVal.append([hpoint, wpoint])
                self.PathSaveCn.append(1)
                print(self.PathSaveVal)
                time.sleep(3)
                boundary = self.cnt2bnd(hpoint, wpoint, self.diameter)
                cv2.rectangle(map, (boundary[0]), (boundary[1]), (225, 225, 225), -1)
                self.gridH = hpoint
                self.gridW = wpoint
                self.GlobalMap = map
            else:
                map = self.GlobalMap
                hpoint = self.gridH
                wpoint = self.gridW

                compass = size.get('orientation', None)
                # print(self.PathSave)
                # print([hpoint, wpoint] in self.PathSave)
                # time.sleep(3)
                if compass != None:
                    compass_cali = self.TPO(compass)
                    if compass_cali == 0:
                        hpoint = self.gridH + self.diameter
                        print('Compass 0 h:', hpoint, "w:", wpoint)
                        # time.sleep(5)
                    elif compass_cali == 90:
                        wpoint = self.gridW + self.diameter
                        print('Compass 90 h:', hpoint, "w:", wpoint)
                        # time.sleep(5)
                    elif compass_cali == 180:
                        hpoint = self.gridH - self.diameter
                        print('Compass 180 h:', hpoint, "w:", wpoint)
                        # time.sleep(5)
                    elif compass_cali == 270:
                        wpoint = self.gridW - self.diameter
                        print('Compass 270 h:', hpoint, "w:", wpoint)

                        # already passed way, and think the robot doesn't move
                        # time.sleep(5)
                boundary = self.cnt2bnd(self.gridH, self.gridW, self.diameter)
                cv2.rectangle(map, (boundary[0]), (boundary[1]), (255, 255, 255), -1)
                cv2.putText(map, str(self.PathSaveCn[self.PathSaveVal.index([self.gridH, self.gridW])]),
                            (self.gridH, self.gridW), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1,
                            lineType=cv2.LINE_AA)

                boundary = self.cnt2bnd(hpoint, wpoint, self.diameter)

                cv2.rectangle(map, (boundary[0]), (boundary[1]), (0, 255, 255), -1)
                self.GlobalMap = map
                # cv2.rectangle(map,(0,0),(10,10),(255,255,255),-1) for testing 0 ,0
                self.gridH = hpoint
                self.gridW = wpoint
                if not ([hpoint, wpoint] in self.PathSaveVal):
                    print("got new points, save them")
                    self.PathSaveVal.append([hpoint, wpoint])
                    self.PathSaveCn.append(1)
                    print(len(self.PathSaveCn), len(self.PathSaveVal))
                else:
                    print("got exist points, update cn")
                    print(len(self.PathSaveCn), ":", len(self.PathSaveVal))
                    index = self.PathSaveVal.index([hpoint, wpoint])
                    print(index)
                    self.addpathCn(index)

        # cv2.imshow("Shinkansan Trace Mapping", map)
        # cv2.waitKey(3)
        else:
            map = self.GlobalMap
            cv2.imshow("Shinkansan Trace Mapping", map)

            # time.sleep(10)
            cv2.waitKey(1)

    def addpathCn(self, index):
        i = self.PathSaveCn[index]
        self.PathSaveCn[index] = i + 1

    def cnt2bnd(self, h, w, bound):
        TopLeft = [int(h + bound / 2), int(w - bound / 2)]
        BottomRight = [int(h - bound / 2), int(w + bound / 2)]
        return [tuple(TopLeft), tuple(BottomRight)]

    def auto_scan(self):
        global boolean01, state01, boolean03, flag01, flag02, flag03, flag04, flag_lost_rotate, flag_lost_cm, flag05, flag06, temp_bb, flag_bb_cm, flag_bb_cm2, flag08
        global flag_obstacle_detected
        temp01 = 0
        temp04 = 0
        #flag_obstacle_detected = 0
        # state01: 0  :  move forward by wall tracing
        #          1  :  front wall is close, turn left
        #          2  :  turn right
        #          3  :  lost wall, move forward and let state01 be 2 to turn right
        #          4  :
        #          6  :  obstacle is detected in my path
        #          7  :  dynamic obstacle is detected in my path
        if keyboard.is_pressed('p'):
            boolean01 = True
            robot.stop()
            time.sleep(1)
            flag02 = False
            flag03 = False
            flag04 = False
            flag01 = True
            flag05 = False
            flag06 = False
            flag08 = False
            flag_lost_rotate = False
            flag_lost_cm = False
            flag_bb_cm = False
            flag_bb_cm2 = False
            return 0
        # for i in range(1,90):
        #    if self._prev_obs.lidar[i] < 80:
        #        print("CLOSE!!!!!!!!!!!!!"+ str(self._prev_obs.lidar[i]))
        #    if self._prev_obs.lidar[360-i] < 100:
        #        print("CLOSE!!!!!!!!!!!!!"+ str(self._prev_obs.lidar[360-i]))

        bb_checked = robot.bb_check()
        # parameter in those, they have no meaning
        if flag04 == True:
            if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43:
                robot.stop()
                flag02 = False
                flag03 = False
                flag04 = False
                flag01 = True
                flag05 = False
                flag06 = False
                flag08 = False
                flag_lost_rotate = False
                flag_lost_cm = False
                flag_bb_cm = False
                flag_bb_cm2 = False
                state01 = 1
            elif bb_checked == True:
                robot.stop()
                flag02 = False
                flag03 = False
                flag04 = False
                flag01 = True
                flag05 = False
                flag06 = False
                flag08 = False
                flag_lost_rotate = False
                flag_lost_cm = False
                flag_bb_cm = False
                flag_bb_cm2 = False
                state01 = 6
            else:
                robot.move_forward_by_cm(1)
            return 0
        elif flag_lost_rotate == True:
            robot.rotate_cw(90)  # only this one have meaning at the first time
            return 0
        elif flag03 == True:  ##MAYBE BUG HERE
            robot.rotate_counter_cw(90)
            return 0
        elif flag_bb_cm == True:
            robot.move_forward_by_cm(temp_bb[0])
            return 0
        elif flag02 == True:
            robot.rotate_cw(90)
            return 0
        elif flag08 == True:
            if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43:
                robot.stop()
                flag02 = False
                flag03 = False
                flag04 = False
                flag01 = True
                flag05 = False
                flag06 = False
                flag08 = False
                flag_lost_rotate = False
                flag_lost_cm = False
                flag_bb_cm = False
                flag_bb_cm2 = False
                state01 = 1
            elif bb_checked == True:
                robot.stop()
                flag02 = False
                flag03 = False
                flag04 = False
                flag01 = True
                flag05 = False
                flag06 = False
                flag08 = False
                flag_lost_rotate = False
                flag_lost_cm = False
                flag_bb_cm = False
                flag_bb_cm2 = False
                state01 = 6
            else:
                robot.move_forward_by_cm2(1)
            return 0
        elif flag_bb_cm2 == True:
            robot.move_forward_by_cm2(temp_bb[1])
            return 0
        elif flag06 == True:
            robot.move_forward_avoid_obs(DRIVE_SPEED)  # only this one have meaning at the first time
            return 0
        elif flag05 == True:
            if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335] <= 43:
                robot.stop()
                flag02 = False
                flag03 = False
                flag04 = False
                flag01 = True
                flag05 = False
                flag06 = False
                flag08 = False
                flag_lost_rotate = False
                flag_lost_cm = False
                flag_bb_cm = False
                flag_bb_cm2 = False
                state01 = 1
            elif bb_checked == True:
                robot.stop()
                flag02 = False
                flag03 = False
                flag04 = False
                flag01 = True
                flag05 = False
                flag06 = False
                flag08 = False
                flag_lost_rotate = False
                flag_lost_cm = False
                flag_bb_cm = False
                flag_bb_cm2 = False
                state01 = 6
            else:
                robot.move_forward_until_wall(DRIVE_SPEED)
            return 0
        elif flag_lost_cm == True:
            robot.move_forward_by_cm(55)  # only this one have meaning at the first time
            return 0

        for i in range(0, 35):  # 35 degree is used after calculating
            if self._prev_obs.lidar[i] < 43:
                temp01 = temp01 + 1
            if i != 0:
                if self._prev_obs.lidar[360 - i] < 43:
                    temp01 = temp01 + 1

        for i in range(0, 35):  # 35 degree is used after calculating
            if self._prev_obs.lidar[i] < 80:
                temp04 = temp04 + 1

            if i != 0:
                if self._prev_obs.lidar[360 - i] < 80:
                    temp04 = temp04 + 1

        # for i
        temp_bb = None
        temp_bb = robot.bb_scan()
        if temp_bb != None and temp_bb[0] == 99999:
            flag_obstacle_detected = 99
            state01 = 7
        elif temp_bb != None:
            state01 = 6
            flag_obstacle_detected = 1

        elif self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43:
            flag_obstacle_detected = 2
            state01 = 1
        elif robot.is_wall_obs() == True:
            flag_obstacle_detected = 0
            state01 = 0
        elif self._prev_obs.lidar[90] > 50:
            flag_obstacle_detected = 3
            state01 = 3
        # elif temp04 > 0 and self._prev_obs.lidar[90] < 80 and self._prev_obs.lidar[270] < 80:
        #    state01 = 5

        # elif temp01 > 0:
        #    for bb in self._prev_obs.boundingbox:
        #        if (bb.origin.z - bb.extent.z) < 5:   # seems to have no margin here, but robot itself has 10 cm height so this is included in margin.
        #           if math.sqrt(bb.origin.x * bb.origin.x + bb.origin.y * bb.origin.y) - math.sqrt(bb.extent.x * bb.extent.x + bb.extent.y * bb.extent.y) < 30:
        #                state01 = 4
        #                break
        ############HERE

        else:
            flag_obstacle_detected = 0
            state01 = 0

        # if temp01 > 30:
        #    state01 = 1
        # elif temp01 > 0 and self._prev_obs.lidar[0] < 140:
        #    state01 = 1

        if state01 == 0:
            robot.compass_trace(DRIVE_SPEED)
        elif state01 == 1:
            robot.rotate_counter_cw(90)
        elif state01 == 2:
            robot.rotate_cw(90)
        elif state01 == 3:
            robot.move_forward_by_cm(
                robot.calculate_cm_after_wall_lost())  # function will calculate adequate dist to go
            flag_lost_rotate = True
            flag05 = True
        elif state01 == 4:
            robot.rotate_counter_cw(90)
            flag06 = True
            # time.sleep(2)
            flag02 = True
        elif state01 == 5:
            robot.rotate_cw(180)
        elif state01 == 6:
            robot.rotate_counter_cw(90)
            flag_bb_cm = True
            flag_bb_cm2 = True
            flag02 = True
        elif state01 == 7:
            robot.move(-30, 0)
            time.sleep(5)
            robot.stop()
            time.sleep(2)
            robot.move(30, 0)
            time.sleep(4)

    def wall_trace(self, velocity: float):
        lidar_front = self._prev_obs.lidar[60]
        lidar_back = self._prev_obs.lidar[120]
        error = lidar_front - lidar_back
        robot.move(velocity, -error * 0.8)

    def move_forward_by_cm(self, distance: float):
        global flag01, state01, flag04, target_distance, initial_encoder, flag_lost_cm, flag_bb_cm
        print("[MOVE FORWARD BY CM]")
        print(flag01)
        if flag01 == True:
            flag01 = False
            flag04 = True
            # initializing
            self.stop()
            target_distance = distance
            initial_encoder = self._prev_obs.encoder.left

        else:
            print("CM!!!!!!!!!!!!!!" + str(target_distance))
            robot.print_current_state()
            time.sleep(0.05)
            current_encoder = self._prev_obs.encoder.left
            delta_distance = abs(current_encoder - initial_encoder) / 36 * 2.411
            target_velocity = abs(target_distance - delta_distance) * 1.2
            if target_velocity <= 20:
                target_velocity = 20
            elif target_velocity > DRIVE_SPEED:
                target_velocity = DRIVE_SPEED
            if abs(delta_distance) >= abs(target_distance):
                self.stop()
                flag01 = True
                flag04 = False
                flag_lost_cm = False
                flag_bb_cm = False
                return True
            else:
                self.compass_trace(target_velocity)
                return False

    def move_forward_by_cm2(self, distance: float):
        global flag01, state01, flag08, target_distance, initial_encoder, flag_lost_cm, flag_bb_cm2
        print("[MOVE FORWARD BY CM]")
        if flag01 == True:  # initializing
            self.stop()
            target_distance = distance
            initial_encoder = self._prev_obs.encoder.left
            flag01 = False
            flag08 = True
        else:
            robot.print_current_state()
            time.sleep(0.05)
            current_encoder = self._prev_obs.encoder.left
            delta_distance = abs(current_encoder - initial_encoder) / 36 * 2.411
            target_velocity = abs(target_distance - delta_distance) * 1.2
            if target_velocity <= 20:
                target_velocity = 20
            elif target_velocity > DRIVE_SPEED:
                target_velocity = DRIVE_SPEED
            if abs(delta_distance) >= abs(target_distance):
                self.stop()
                flag01 = True
                flag08 = False
                flag_lost_cm = False
                flag_bb_cm2 = False
                return True
            else:
                self.compass_trace(target_velocity)
                return False

    def move_forward_until_wall(self, velocity: float):
        global flag01, state01, flag05
        print("[MOVE FORWARD UNTIL WALL]")
        if flag01 == True:  # initializing
            self.stop()
            flag01 = False
            flag05 = True
        else:
            robot.print_current_state()
            time.sleep(0.05)
            current_lidar = self._prev_obs.lidar[90]
            if current_lidar < 70:
                self.stop()
                flag01 = True
                flag05 = False
                return True
            else:
                self.move(velocity, 0)
                return False

    def move_forward_avoid_obs(self, velocity: float):
        global flag01, state01, flag06
        print("[MOVE FORWARD AVOID OBS]")
        if flag01 == True:  # initializing
            self.stop()
            flag01 = False
            flag06 = True
        else:
            robot.print_current_state()
            time.sleep(0.05)
            temp02 = 0
            for i in range(0, 180):
                if self._prev_obs.lidar[i] < 45:
                    temp02 = temp02 + 1
            if temp02 == 0:
                self.stop()
                flag01 = True
                flag06 = False
                return True
            else:
                self.move(velocity, 0)
                return False

    def TPO(self, input01: float):     ## 현재 각도 90도 단위로 근사하는 함수, 근사값 반환
        current_compass = abs(input01)
        if abs(current_compass - 0) < 45 or abs(current_compass - 360) < 45:
            current_compass = 0
        elif abs(current_compass - 90) < 45:
            current_compass = 90
        elif abs(current_compass - 180) < 45:
            current_compass = 180
        elif abs(current_compass - 270) < 45:
            current_compass = 270
        return current_compass

    # Tight TPO hehe
    def TTPO(self, input01: float):
        current_compass = abs(input01)
        if abs(current_compass - 0) < 7 or abs(current_compass - 360) < 7:
            current_compass = 0
        elif abs(current_compass - 90) < 7:
            current_compass = 90
        elif abs(current_compass - 180) < 7:
            current_compass = 180
        elif abs(current_compass - 270) < 7:
            current_compass = 270
        return current_compass

    def compass_trace(self, velocity: float):
        current_compass = self._prev_obs.compass
        error = current_compass - self.TPO(current_compass)
        if error > 300:
            error = error - 360
        print(current_compass)
        print(self.TPO(current_compass))
        self._cls()
        robot.move(velocity, error * 1.0)

    def bb_scan(self):
        for bb in self._prev_obs.boundingbox:
            if (
                    bb.origin.z - bb.extent.z) < 5:  # seems to have no margin here, but robot itself has 10 cm height so this is included in margin.
                if abs(robot.TPO(bb.rotation.z) - robot.TPO(self._prev_obs.compass)) % 180 == 0:
                    width = bb.extent.y
                    height = bb.extent.x
                else:
                    width = bb.extent.x
                    height = bb.extent.y
                if bb.origin.x > 0 and (bb.origin.x - height) < 43 and (17 + width + 4) > abs(bb.origin.y):
                    # if abs(robot.TPO(bb.rotation.z)) % 180 == 0:
                    print("WIDTH:  " + str(width))
                    print("HEIGHT:  " + str(height))
                    print("ORIGIN:  " + str(bb.origin.y))
                    print(-1 * bb.origin.y + width + 17 + 10)
                    # time.sleep(5)
                    if bb.class_id == 20:
                        return [99999, 99999]  # dynamic obstacle
                    elif abs(abs(bb.rotation.z) - robot.TPO(bb.rotation.z)) < 10:
                        return [(-1 * bb.origin.y + width + 17 + 25), (34 + 25 + height)]
                    else:
                        # time.sleep(10)
                        return [(-1 * bb.origin.y + width + 17 + 50), (34 + 45 + height)]
        else:
            return None

    def bb_check(self):
        for bb in self._prev_obs.boundingbox:
            if (
                    bb.origin.z - bb.extent.z) < 5:  # seems to have no margin here, but robot itself has 10 cm height so this is included in margin.
                if abs(robot.TPO(bb.rotation.z) - robot.TPO(self._prev_obs.compass)) % 180 == 0:
                    width = bb.extent.y
                    height = bb.extent.x
                else:
                    width = bb.extent.x
                    height = bb.extent.y
                if bb.origin.x > 0 and (bb.origin.x - height) < 38 and (17 + width + 3) > abs(bb.origin.y):
                    return True
        else:
            return None

    def calculate_cm_after_wall_lost(self):
        closest_obs_dist = 60000
        closest_obs_deg = 0
        for i in range(90, 180):
            if self._prev_obs.lidar[i] < closest_obs_dist:
                closest_obs_dist = self._prev_obs.lidar[i]
                closest_obs_deg = i
        if (17 + 37 - math.sin((closest_obs_deg - 90) * math.pi / 180) * closest_obs_dist) > 0:
            return (17 + 37 - math.sin((closest_obs_deg - 90) * math.pi / 180) * closest_obs_dist)
        else:
            return 0

    def is_wall_obs(self):
        for bb in self._prev_obs.boundingbox:
            if (bb.origin.z - bb.extent.z) < 5:
                if abs(robot.TPO(bb.rotation.z) - robot.TPO(self._prev_obs.compass)) % 180 == 0:
                    width = bb.extent.y
                    height = bb.extent.x
                else:
                    width = bb.extent.x
                    height = bb.extent.y
                if bb.origin.y > 0 and (bb.origin.y - width) < 48 and (abs(bb.origin.x) < (height + 10)):
                    print("OBS!")
                    return True
        return False

    def search_start_point(self):
        global flag_search_start_point

        while True:  ## yaw = 0도 될 때까지 cw 회전
            if robot.TPO(self._prev_obs.compass) == 0:
                break
            self.rotate_cw(90)
            while not self.rotate_cw(90):
                pass   ##pass항으로 들어오는 경우 robot.TPO(self._prev_obs.compass)==0인 경우 아닐까 생각

        while True:
            print("SEARCH START POINT")
            print(self._prev_obs.compass)
            print(robot.is_wall_obs())
            robot._cls()
            robot.auto_scan()
            if (((360 - self._prev_obs.compass) < 2 or self._prev_obs.compass < 2) and (
                    self._prev_obs.lidar[90] < 45 or (robot.is_wall_obs() == True))):
                ## yaw가 2도의 오차범위로 0도와 가깝고, 우측에 벽이 존재하거나 _prev_obs.lidar[90]<45일 때
                flag_search_start_point = False
                robot.stop()
                break
        print("NOW START AUTO SCANNING PROSEDURE")
        robot.TraceMapping(orientation=self._prev_obs.compass)
        robot.TraceMapping(showmode=True)
        robot.stop()
        time.sleep(0.2)

    def auto_scan_for_cm(self, dist: float):
        global flag_auto_scan_for_cm, initial_encoder02, desired_dist02, current_dist02, flag_obstacle_detected, flag_rest_region_cleaned, flag_center_point_mapping, init_encoder, flag01
        # flag_obstacle_detected :  1 :  obstacle to avoid
        #                          2 :  normal wall in front of me
        #                          3 :  wall lost
        if flag_auto_scan_for_cm == True:
            initial_encoder02 = self._prev_obs.encoder.left
            flag_auto_scan_for_cm = False
            desired_dist02 = dist - 2
            flag_center_point_mapping = False
            time.sleep(0.1)
        else:
            current_encoder = self._prev_obs.encoder.left
            current_dist02 = (current_encoder - initial_encoder02) / 36 * 2.411
            ## auto scan 시작할 때부터 달린 거리
            if flag_center_point_mapping == False and current_dist02 > 34 and abs(current_dist02 - 34) < 2:
                flag_center_point_mapping = True
                robot.TraceMapping(orientation=self._prev_obs.compass)
                robot.TraceMapping(showmode=True)
            if current_dist02 >= desired_dist02:
                robot.stop()
                flag_auto_scan_for_cm = True
                print("MOVED 68CM")
                time.sleep(0.1)
                robot.TraceMapping(orientation=self._prev_obs.compass)
                robot.TraceMapping(showmode=True)
                robot.initialize_all_flags()
                if robot.Hansel_and_Gretel() == False:
                    robot.clean_room_basic(33)
                else:
                    print("SNACK WAS THERE")
                    time.sleep(1)
                return False
            else:
                flag_obstacle_detected = 0
                robot.auto_scan()
                print("##################FLAG OBSTACLE DETECTED:  " +str(flag_obstacle_detected))
                if flag_obstacle_detected == 99:
                    flag_obstacle_detected = 0
                if flag_obstacle_detected != 0:

                    # something was detected while scanning for 68 cm region
                    if flag_obstacle_detected == 99:
                        flag_obstacle_detected = 0
                        robot.move(-30, 0)
                        time.sleep(5)
                        robot.stop()
                        time.sleep(2)
                        robot.stop()
                        robot.move(30, 0)
                        time.sleep(4)
                    elif flag_rest_region_cleaned == False and current_dist02 > 30:
                        # the region was not cleaned yet. let's clean the region by short dist (current_dist02)
                        # USE current_dist02
                        flag_obstacle_detected = 0
                        print("LET's CLEAN THIS SMALL AREA")
                        flag_auto_scan_for_cm = True
                        flag_rest_region_cleaned = True
                        robot.stop()
                        time.sleep(0.1)
                        # robot.TraceMapping(orientation=self._prev_obs.compass)
                        # robot.TraceMapping(showmode=True)
                        robot.initialize_all_flags()
                        if robot.Hansel_and_Gretel() == False:

                            robot.clean_room_basic(current_dist02)
                        else:
                            print("SNACK WAS THERE")
                            time.sleep(0.1)
                        return True
                    else:
                        # the region was cleaned already or too small that could ignore here. let's find another place
                        # robot.TraceMapping(orientation=self._prev_obs.compass)
                        # robot.TraceMapping(showmode=True)
                        flag_obstacle_detected = 0
                        flag_rest_region_cleaned = False
                        while robot.TPO(self._prev_obs.compass) == 0 or robot.TPO(self._prev_obs.compass) == 180:
                            print("LETS FIND ANOTHER PLACE11111")
                            robot.auto_scan()
                        robot.stop()
                        print("1ST")
                        print(self._prev_obs.compass)
                        print(robot.TPO(self._prev_obs.compass))
                        #time.sleep(1)
                        while robot.TPO(self._prev_obs.compass) == 90 or robot.TPO(self._prev_obs.compass) == 270:
                            print("LETS FIND ANOTHER PLACE222222")
                            robot.auto_scan()
                            if (robot.TTPO(self._prev_obs.compass) == 90 or robot.TTPO(self._prev_obs.compass) == 270):
                                break

                        robot.stop()
                        print("2nd")
                        #time.sleep(1)

                        while not (
                                robot.TTPO(self._prev_obs.compass) == 90 or robot.TTPO(self._prev_obs.compass) == 270):
                            print("LETS FIND ANOTHER PLACE333333")
                            robot.auto_scan()
                        robot.stop()
                        print("2nd- 2nd")
                        #time.sleep(1)
                        robot.move(30,0)
                        time.sleep(2)
                        init_encoder = self._prev_obs.encoder.left
                        #flag01 = True
                        while True:
                            print("LETS FIND ANOTHER PLACE4444444")
                            later_encoder = self._prev_obs.encoder.left
                            delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
                            if delta_encoder >= 34 and (delta_encoder - 34) < 2:
                                robot.TraceMapping(orientation=self._prev_obs.compass)
                                robot.TraceMapping(showmode=True)
                                init_encoder = self._prev_obs.encoder.left
                            self.compass_trace(DRIVE_SPEED)
                            if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                                25] <= 43 or self._prev_obs.lidar[335
                            ] <= 43 or robot.bb_scan() != None or robot._prev_obs.lidar[90] > 120:
                                break
                        robot.stop()

                        if self._prev_obs.lidar[90] > 120:
                            robot.move(30,0)
                            time.sleep(1)
                            robot.stop()
                            flag01 = True
                            self.rotate_cw(90)
                            while not self.rotate_cw(90):
                                print("RIGHT WALL IS MISSING")
                                pass
                        else:
                            flag01 = True
                            self.rotate_counter_cw(90)
                            while not self.rotate_counter_cw(90):
                                print("RIGHT WALL IS EXIST")
                                pass


                        print("3rd")
                        time.sleep(0.1)
                        # move forward until wall
                        init_encoder = self._prev_obs.encoder.left
                        while not (robot.is_wall_obs() == True or self._prev_obs.lidar[90] < 55):
                            later_encoder = self._prev_obs.encoder.left
                            delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
                            if delta_encoder >= 34 and (delta_encoder - 34) < 2:
                                robot.TraceMapping(orientation=self._prev_obs.compass)
                                robot.TraceMapping(showmode=True)
                                init_encoder = self._prev_obs.encoder.left
                            print("LETS FIND ANOTHER PLACE555555")
                            print("MOVE UNTIL WALL")
                            self.compass_trace(DRIVE_SPEED)
                            if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                                25] <= 43 or self._prev_obs.lidar[335
                            ] <= 43 or robot.bb_scan() != None:
                                break
                        robot.stop()
                        #time.sleep(1)
                        print("NOW START COUNTING ENCODER AGAIN!!!!")
                        robot.stop()

                        flag_auto_scan_for_cm = True
                        return False
                return False

    def initialize_all_flags(self):
        global boolean01, state01, boolean03, flag01, flag02, flag03, flag04, flag_lost_rotate, flag_lost_cm, flag05, flag06, temp_bb, flag_bb_cm, flag_bb_cm2, flag08
        global flag_obstacle_detected
        flag02 = False
        flag03 = False
        flag04 = False
        flag01 = True
        flag05 = False
        flag06 = False
        flag08 = False
        flag_lost_rotate = False
        flag_lost_cm = False
        flag_bb_cm = False
        flag_bb_cm2 = False

    def clean_room_basic(self, input99: float):
        global flag01
        count_34cm = 0
        self.rotate_counter_cw(90)
        while not self.rotate_counter_cw(90):
            print("CLEAN ROOM MODE")
            pass
        init_encoder = self._prev_obs.encoder.left
        later_encoder = init_encoder
        delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
        temp_bb = robot.bb_scan()
        while True:  # obs_state !=True and
            temp_bb = robot.bb_scan()
            print("CLEAN ROOM MODE")
            if temp_bb != None and temp_bb[0] == 99999:
                robot.move(-30, 0)
                time.sleep(5)
                robot.stop()
                time.sleep(2)
                robot.stop()
                robot.move(30, 0)
                time.sleep(4)
                init_encoder = self._prev_obs.encoder.left
                continue
            temp_bb = robot.bb_scan()
            later_encoder = self._prev_obs.encoder.left
            delta_encoder = (later_encoder - init_encoder) / 36 * 2.411
            if delta_encoder >= 34 and (delta_encoder - 34) < 2:
                count_34cm = count_34cm + 1
                robot.TraceMapping(orientation=self._prev_obs.compass)
                robot.TraceMapping(showmode=True)
                init_encoder = self._prev_obs.encoder.left
                later_encoder = init_encoder
                delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
            if (self._prev_obs.lidar[0] <= 41 or self._prev_obs.lidar[
                25] <= 43 or self._prev_obs.lidar[335
                ] <= 43 or self.bb_check() == True):
                break
            self.compass_trace(DRIVE_SPEED)
            print("lidar[0] : ", self._prev_obs.lidar[0])
            print("delta_encode : ", delta_encoder)
            self._cls()
        robot.stop()
        print("fffffffff")

        self.rotate_counter_cw(90)
        while not self.rotate_counter_cw(90):
            print("CLEAN ROOM MODE")
            pass
        robot.stop()
        robot.TraceMapping(orientation=self._prev_obs.compass)
        robot.TraceMapping(showmode=True)
        init_encoder = self._prev_obs.encoder.left
        later_encoder = init_encoder
        delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
        print(self.bb_check)
        time.sleep(0.1)
        while not (delta_encoder >= input99 - 1 or self._prev_obs.lidar[0] <= 41 or self._prev_obs.lidar[
            25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43 or self.bb_check() == True):  # obs_state !=True and
            print("CLEAN ROOM MODE")
            self.compass_trace(DRIVE_SPEED)
            later_encoder = self._prev_obs.encoder.left  # update the later value
            delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
            print("lidar[0] : ", self._prev_obs.lidar[0])
            print("delta_encode : ", delta_encoder)
            self._cls()
        robot.stop()
        self.rotate_counter_cw(90)
        while not self.rotate_counter_cw(90):
            print("CLEAN ROOM MODE")
            pass
        robot.stop()

        ### if meet a obstacle, avoid the obs and return to original position
        init_encoder = self._prev_obs.encoder.left
        later_encoder = init_encoder
        delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
        temp_bb = robot.bb_scan()
        print(self.bb_check)
        time.sleep(0.1)
        while True:  # obs_state !=True and
            print("CLEAN ROOM MODE")
            temp_bb = robot.bb_scan()
            if temp_bb != None and temp_bb[0] == 99999:
                robot.move(-30, 0)
                time.sleep(5)
                robot.stop()
                time.sleep(2)
                robot.move(30, 0)
                time.sleep(4)
                robot.stop()
                init_encoder = self._prev_obs.encoder.left
                continue
            temp_bb = robot.bb_scan()
            later_encoder = self._prev_obs.encoder.left
            delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
            if delta_encoder >= 34 and (delta_encoder - 34) < 2:
                count_34cm = count_34cm - 1
                robot.TraceMapping(orientation=self._prev_obs.compass)
                robot.TraceMapping(showmode=True)
                init_encoder = self._prev_obs.encoder.left
                later_encoder = init_encoder
                delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
            if (self._prev_obs.lidar[0] <= 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43 or self.bb_check() == True):
                break
            self.compass_trace(DRIVE_SPEED)
            print("lidar[0] : ", self._prev_obs.lidar[0])
            print("delta_encode : ", delta_encoder)
            self._cls()
        robot.stop()
        print("fffffffff")
        if count_34cm > 6:
            # return mode
            self.rotate_counter_cw(90)
            while not self.rotate_counter_cw(90):
                print("CLEAN ROOM MODE")
                print("RETURN MODE!!!!!!!!!!!!!!!!!!!!!")
                pass
            robot.stop()
            # move forward until no obstacle on the right
            while (self._prev_obs.lidar[90] < 40 or self.is_wall_obs() == True):
                print("CLEAN ROOM MODE")
                print("RETURN MODE!!!!!!!!!!!!!!!!!!!!!")
                robot.compass_trace(DRIVE_SPEED)
                if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43 or robot.bb_scan() != None:
                    break
            robot.initialize_all_flags()
            self.move_forward_by_cm(30)
            while not self.move_forward_by_cm(1):
                print("CLEAN ROOM MODE")
                print("RETURN MODE!!!!!!!!!!!!!!!!!!!!!")
                if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43:
                    break
                pass
            robot.TraceMapping(orientation=self._prev_obs.compass)
            robot.TraceMapping(showmode=True)

            robot.initialize_all_flags()

            self.rotate_cw(90)
            while not self.rotate_cw(90):
                print("CLEAN ROOM MODE")
                print("RETURN MODE!!!!!!!!!!!!!!!!!!!!!")
                pass
            init_encoder = self._prev_obs.encoder.left
            while not (self._prev_obs.lidar[0] <= 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43 or self.bb_check() == True):  # obs_state !=True and
                print("CLEAN ROOM MODE")
                print("RETURN MODE!!!!!!!!!!!!!!!!!!!!!")
                later_encoder = self._prev_obs.encoder.left
                delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
                if delta_encoder >= 34 and (delta_encoder - 34) < 2:
                    count_34cm = count_34cm - 1
                    robot.TraceMapping(orientation=self._prev_obs.compass)
                    robot.TraceMapping(showmode=True)
                    init_encoder = self._prev_obs.encoder.left
                    later_encoder = init_encoder
                    delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
                self.compass_trace(DRIVE_SPEED)
                print("lidar[0] : ", self._prev_obs.lidar[0])
                print("delta_encode : ", delta_encoder)
                self._cls()
            #newly added
            self.rotate_counter_cw(90)
            while not self.rotate_counter_cw(90):
                print("CLEAN ROOM MODE")
                print("RETURN MODE!!!!!!!!!!!!!!!!!!!!!")
                pass
            robot.stop()
            # move forward until wall
            while not (robot.is_wall_obs() == True or self._prev_obs.lidar[90] < 50):
                print("CLEAN ROOM MODE")
                print("MOVE UNTIL WALL")
                print("RETURN MODE!!!!!!!!!!!!!!!!!!!!!")
                self.compass_trace(DRIVE_SPEED)
                if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                    25] <= 43 or self._prev_obs.lidar[335
                ] <= 43 or robot.bb_scan() != None:
                    break
            robot.stop()
        else:
            self.rotate_counter_cw(90)
            while not self.rotate_counter_cw(90):
                print("CLEAN ROOM MODE")
                pass
            robot.stop()
            robot.TraceMapping(orientation=self._prev_obs.compass)
            robot.TraceMapping(showmode=True)
            init_encoder = self._prev_obs.encoder.left
            later_encoder = init_encoder
            delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
            while not (delta_encoder >= input99 - 1 or self._prev_obs.lidar[0] <= 41 or self._prev_obs.lidar[
                25] <= 39 or  self._prev_obs.lidar[335
                ] <= 39 or self.bb_check() == True):  # obs_state !=True and
                print("CLEAN ROOM MODE")
                self.compass_trace(DRIVE_SPEED)
                later_encoder = self._prev_obs.encoder.left  # update the later value
                delta_encoder = abs(later_encoder - init_encoder) / 36 * 2.411
                print("lidar[0] : ", self._prev_obs.lidar[0])
                print("delta_encode : ", delta_encoder)
                self._cls()
            if delta_encoder < 10:
                self.rotate_counter_cw(90)
                while not self.rotate_counter_cw(90):
                    print("CLEAN ROOM MODE")
                    print("FIND NEW AREA MODE!!!!!!!!!!!!!")
                    pass
                robot.stop()
                time.sleep(0.1)
                # move forward until no wall
                while (robot.is_wall_obs() == True or self._prev_obs.lidar[90] < 41):
                    print("CLEAN ROOM MODE")
                    print("FIND NEW AREA MODE!!!!!!!!!!!!!")
                    self.compass_trace(DRIVE_SPEED)
                    if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43 or robot.bb_scan() != None:
                        break

                robot.initialize_all_flags()

                self.move_forward_by_cm(24)
                while not self.move_forward_by_cm(1):
                    print("CLEAN ROOM MODE")
                    print("FIND NEW AREA MODE!!!!!!!!!!!!!")
                    if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43 or robot.bb_scan() != None:
                        break
                    pass

                robot.initialize_all_flags()

                robot.stop()
                time.sleep(0.1)
                self.rotate_cw(90)
                while not self.rotate_cw(90):
                    print("CLEAN ROOM MODE")
                    print("FIND NEW AREA MODE!!!!!!!!!!!!!")
                    pass
                robot.stop()
                time.sleep(0.1)

                self.move_forward_by_cm(34)
                while not self.move_forward_by_cm(1):
                    print("CLEAN ROOM MODE")
                    print("FIND NEW AREA MODE!!!!!!!!!!!!!")
                    if self._prev_obs.lidar[0] < 41 or self._prev_obs.lidar[
                25] <= 43 or  self._prev_obs.lidar[335
                ] <= 43 or robot.bb_scan() != None:
                        break
                    pass

                robot.initialize_all_flags()

            robot.stop()
            time.sleep(0.1)

    def stop_detecter(self):
        global ir_315, ir_45
        ir_315.append(self._prev_obs.ir[0])
        ir_45.append(self._prev_obs.ir[4])
        flag_det_45 = True
        flag_det_315 =True
        if (len(ir_315) >= 100):
            del ir_315[0]
        if (len(ir_45) >= 100):
            del ir_45[0]

        for i in range(0, len(ir_315)):
            if ir_315[i] != self._prev_obs.ir[0]:
                flag_det_315 = False  # it was moved
                break
            elif ir_315[i] > 20:
                flag_det_315 = False
                break
            elif ir_45[i] != self._prev_obs.ir[4]:
                flag_det_45 = False  # it was moved
                break
            elif ir_45[i] > 20:
                det_45 = False
                break
        if (flag_det_315 == True and flag_det_45 == True):
            return True  # it is stucked
        else:
            return False


if __name__ == "__main__":
    print("[BEGIN]")

    robot = RobotController()
    env = Env()
    input01 = ''
    boolean01 = False
    state01 = 0
    flag01 = True  # initializing
    flag02 = False  # rotating
    flag03 = False  # rotating
    flag04 = False  # move_forward_by_centimeter
    flag05 = False  # move_forward_until_wall
    flag06 = False  # move_forward_aviod_obs
    flag08 = False  # move_forward_by_cm 2
    flag_lost_rotate = False
    flag_lost_cm = False
    flag_boundingbox_check = False
    flag_bb_cm = False
    flag_bb_cm2 = False
    flag_search_start_point = True
    flag_auto_scan_for_cm = True
    current_dist = 0
    flag_obstacle_detected = False
    flag_rest_region_cleaned = False
    init_encoder = 0
    ir_315 =[]
    ir_45 = []
    while True:
        try:
            robot.print_current_state()
            print(env.wheel_radius)  # input01 = msvcrt.getch()
            # if keyboard.is_pressed('w') or keyboard.is_pressed('s') or keyboard.is_pressed('a') or keyboard.is_pressed('d'):
            # input01 = msvcrt.getch()
            # input01 = input01.decode('UTF-8')
            # x = robot._prev_obs.boundingbox

            if flag02:
                print("FLAG02")
            if flag03:
                print("FLAG03")
            if flag04:
                print("FLAG04")
            if flag05:
                print("FLAG05")
            if flag06:
                print("FLAG06")
            if flag08:
                print("FLAG08")
            if flag_bb_cm:
                print("FLAG_BB_CM")
            if flag_bb_cm2:
                print("FLAG_BB_CM2")
            if flag_lost_rotate:
                print("FLAG LOST ROTATE")
            # if flag_lost_cm:
            #    print("FLAG LOST CM")
            if boolean01 == True:
                print("AUTO MODE")
            if boolean01 == False:
                robot.easy_control()

            else:
                if flag_search_start_point == True:
                    robot.search_start_point()
                    boolean01 = True
                else:
                    boolean01 = True
                    robot.auto_scan_for_cm(68)
                    if robot.stop_detecter() == True:
                        robot.stop()
                        print("ROBOT IS ON STUCKING")
                        time.sleep(3)
                        robot.move(-30,0)
                        time.sleep(1)
                        robot.initialize_all_flags()
                        flag_search_start_point = True
                        robot.stop()

                    boolean01 = True
            pass
        except RuntimeError as e:
            print(e)
        except KeyboardInterrupt:
            break

    print("[END]")
