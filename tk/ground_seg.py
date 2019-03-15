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
import math

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
        #print(depth_image[360][550], len(depth_image[0]), str(depth_image[360][550] * depth_scale) + "m")

        depth_image2 = depth_image.copy()
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        #images = np.hstack((color_image, depth_colormap))
        images = color_image

        #get [i,640] column
        _640col = [a[640] for a in depth_image2]
        #print('1111111111', _640col)

        abs_x = []
        abs_y = []
        ground_center_idx = []

        #depth_image2[360] = depth_image2[360] * float(depth_scale)
        for idx, temp in enumerate(_640col):
            if _640col[idx] == 0:
                abs_x.append(None)
                abs_y.append(None)
            else:
                _640col[idx] = temp * depth_scale
                abs_x.append(temp * depth_scale*math.cos(((float)(58.0/2.0-58.0*(float)(idx)/720.0))*3.14/180.0))
                abs_y.append(temp * depth_scale * math.sin(float(58.0/2.0 - 58.0 * (float)(idx) / 720.0)*3.14/180.0))

            #print((float)(58.0 - 58.0 * (float)(idx) * 2.0 / 720.0))
            #print(math.cos((float)(58.0 - 58.0 * (float)(idx) * 2.0 / 720.0)))
            #print(temp * depth_scale*math.cos((float)(58.0-58.0*(float)(idx)*2.0/720.0)))
            #print(temp * depth_scale * math.sin((float)(58.0 - 58.0 * (float)(idx) * 2.0 / 720.0)))
            #print(idx, abs_x[idx], abs_y[idx])
            #print(type(depth_image2[360][idx]))
        #print("1212" + str(depth_image2[360][12]))
        #y = list(range(0,1280))
        #print(len(y))
        idx=0
        while abs_x[720-30-idx]==None:
            idx+=1
            #print("FUCKFUCK")
        ground_center_idx.append(720-30-idx)
        i=0
        idx=31
        while idx<720:
            try:
                if abs_x[720-idx] == None:
                    idx+=5
                    #print("FUCK")
            #(abs( abs_x[ground_center_idx[i]] - abs_x[(720-idx)]) < 0.15 ) and

                elif (abs( abs_x[ground_center_idx[i]] - abs_x[(720-idx)]) < 0.4 ) and (abs( abs_y[ground_center_idx[0]] - abs_y[(720-idx)])<0.05):
                    ground_center_idx.append((720-idx))
                    i += 1
                    idx += 5
                    cv2.circle(images, (640, (720-idx)), 5, (0, 255, 0), 5)
                    #print(idx)
                    #print("FUCKFUCKFUCK")
                else:
                    idx+=5

            except:
                print("FOUND FUCKING NONE")

        #plt.plot(abs_x, abs_y)
        #plt.xlim(0, 2)
        #plt.ylim(-2, 2)

        #plt.pause(0.05)
        #plt.cla()
        #plt.clf()

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)5
        cv2.circle(images, (640, 690), 5, (255, 255, 255), 10)
        cv2.putText(images, str(_640col[600]) + "m", (640, 690), font, fontScale, yellow, 2 )
        cv2.line(images, (640, 0), (640, 720), (0, 0, 0), 5)
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)




finally:

    # Stop streaming
    pipeline.stop()
