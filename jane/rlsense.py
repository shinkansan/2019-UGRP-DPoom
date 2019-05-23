######################################
#######realsense plotting#############
######################################
'''
Working with
	UBUNTU 16.04 LTS
	OPENCV 4.0~
	Python 3.6
	pyrealsense2
	matplotlib
	numpy
	for intel D435i
'''

import pyrealsense2 as rs
import numpy as np
import cv2

import matplotlib.pyplot as plt

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)
font = cv2 .FONT_HERSHEY_SCRIPT_SIMPLEX
fontScale = 1.5
yellow = (0,255,255)

st_pt = ed_pt = 0
dr_up = dr_down = 360
counting = 0 ##
up = down = 360

'''Robot '''
robot_width = 50

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data(), dtype=float)
        color_image = np.asanyarray(color_frame.get_data())

        flag_srch_obj = False
        flag_found_obj = False
        flag_box = False
        check = [[0] * 1500 for i in range(3)]

        #print(depth_image[360][550], len(depth_image[0]), str(depth_image[360][550] * depth_scale) + "m")

        depth_image2 = depth_image.copy()
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        #images = np.hstack((color_image, depth_colormap))
        images = color_image


        ###################Filter###############
        for i, val in enumerate(depth_image2[360]):

            if val*depth_scale >= 0.5:    ####>2.0
                depth_image2[360][i] = 2.0/float(depth_scale)
                ##print(depth_image[360][12])
                if flag_srch_obj and int(i+(i-640)*0.5) <= 1500:
                    check[0][i] = 2
                    #print('>2 : val*depth_scale is', val*depth_scale, 'end point check is : ', i, '->', int(i+(i-640)*0.5))
                    check[1][i] = up
                    check[2][i] = down
                    up = down = 360
                    flag_srch_obj = False

            elif val*depth_scale == 0:
                depth_image2[360][i] = None
                ##print(depth_image[360][12])
                if flag_srch_obj and int(i+(i-640)*0.5) <= 1500:
                    check[0][i] = 2
                    #print('0 : val*depth_scale is', val * depth_scale, 'end point check is : ', i, '->', int(i+(i-640)*0.5))
                    check[1][i] = up
                    check[2][i] = down
                    up = down = 360
                    flag_srch_obj = False

            elif val*depth_scale < 0.5:
                cv2.circle(images, (int((i+(i-640)*0.5)), 360), 5, (0, 0, 255), 25)
                #print('<0.5 point! ', i)
                flag_found_obj = True
                if flag_srch_obj is False and int(i+(i-640)*0.5) <= 1500:
                    check[0][i] = 1
                    #print('<0.5 : val*depth_scale is', val * depth_scale, 'start point check is : ', i, '->',
                    #     int(i + (i - 640) * 0.5))
                    flag_srch_obj = True

                j = down
                try:  # 아래
                    while depth_image2[j][i] * depth_scale < 0.5 and depth_image2[j][
                        i] * depth_scale != 0 and j <= 720:
                        if j >= down:
                            down = j
                        j = j + 1
                except:
                    continue
                j = up
                try:
                    while depth_image2[j][i] * depth_scale < 0.5 and depth_image2[j][
                        i] * depth_scale != 0 and j >= 0:
                        if j <= up:
                            up = j
                        j = j - 1
                except:
                    continue

        if flag_found_obj:
            dr_up = dr_down = 360
            flag_box = False

            for i, val in enumerate(depth_image2[360]):
                if check[0][i] == 1:
                    if flag_box:
                        if int(i+(i-640)*0.5)-ed_pt > 200:
                            '''
                            drr_up = dr_up
                            drr_down = dr_down
                            stt_pt = st_pt
                            edd_pt = ed_pt
                            '''
                            cv2.rectangle(images, (st_pt, dr_up), (ed_pt, dr_down), (0, 255, 0), 3)
                            #print('st_pt :', st_pt, 'ed_pt :', ed_pt)
                            #if st_pt > ed_pt:
                                #print('st_pt is bigger than ed_pt!')
                            cv2.putText(images, 'Object', (int((st_pt+ed_pt)/2), 360), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255))
                            counting = counting+1
                            #print('Counting :', counting)
                            dr_up = dr_down = 360
                            st_pt = int(i+(i-640)*0.5)
                            #print('st_pt1 is :', st_pt)
                            flag_box = False
                    else:
                        st_pt = int(i+(i-640)*0.5)
                        #print('st_pt2 is :', st_pt)
                elif check[0][i] == 2:
                    ed_pt = int(i+(i-640)*0.5)
                    #print('ed_pt is : ', ed_pt)
                    #print('up is', check[1][i], 'down is', check[2][i])
                    if check[1][i] <= dr_up:
                        dr_up = check[1][i]
                    if check[2][i] >= dr_down:
                        dr_down = check[2][i]
                    #print('dr_down is', dr_down, 'dr_up is', dr_up)
                    flag_box = True
            cv2.rectangle(images, (st_pt, dr_up), (ed_pt, dr_down), (0, 255, 0), 3)
            #print('last st_pt :', st_pt, 'last ed_pt :', ed_pt)
            cv2.putText(images, 'Object', (int((st_pt+ed_pt)/2), 360), cv2.FONT_HERSHEY_SIMPLEX, 2,
                        (255, 255, 255))

        '''
        #depth_image2[360] = depth_image2[360] * float(depth_scale)
        for idx, temp in enumerate(depth_image2[360]):
            depth_image2[360][idx] = temp * depth_scale
            print(depth_image2[360][idx])
            print(type(depth_image2[360][idx]))
        #print("1212" + str(depth_image2[360][12]))
        y = list(range(0,1280))
        print(len(y))
        #plt.plot(y, depth_image2[360])
        #plt.show()
        '''



        #plt.pause(0.05)
        #plt.cla()
        #plt.clf()
        # Apply icolormap on depth image (image must be converted to 8-bit per pixel first)

        cv2.circle(images, (1280+550, 360), 5, (255, 255, 255), 50)
        cv2.putText(images, str(depth_image[360][550] * depth_scale) + "m", (1280+550, 360), font, fontScale, yellow, 5 )
        cv2.line(images, (0, 360), (2560, 360), (0, 0, 0), 5)
        # Show images

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)




finally:

    # Stop streaming
    pipeline.stop()