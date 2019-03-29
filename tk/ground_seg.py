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
import time

global depth_scale, ROW, COL

#size of images
COL= 720
ROW = 1280

VERTICAL_CORRECTION = 0.15 #0.45  #parameter of correction for parabola to linear
WARP_PARAM = 0.45  #value should be 0.0 ~ 1.0. Bigger get more warped.
GRN_ROI = 400 #The index of col that want to consider as ground ROI
ZOOM_PARAM = 0.15 #Force calibrating the depth image to match the color image
font = cv2.FONT_HERSHEY_SCRIPT_SIMPLEX
fontScale = 1.5
yellow = (0, 255, 255)


#Topview image. src, dst are numpy array.
#########################Move LAVACON TO EACH EDGES AND TRY AGAING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
def Topview(src):
    global WARP_PARAM, ROW, COL
    # col=720, row=1280
    col, row = src.shape[0], src.shape[1]

    corners = np.float32([[row*WARP_PARAM/2, 0], [row*(1-WARP_PARAM/2), 0], [0, col], [row, col]])
    warp_corners = np.float32([[0, 0], [ROW, 0], [0, COL], [ROW, COL]])
    #print(corners)
    #print(warp_corners)

    trans_matrix = cv2.getPerspectiveTransform(corners, warp_corners)

    dst = cv2.warpPerspective(src, trans_matrix, (ROW, COL))
    return dst


#vertically scan ground
def verticalGround(depth_image2, images, numCol, plot):
    global depth_scale, GRN_ROI, ROW, COL

    ###################################################################################
    #############Calibration. CHECK HERE WHEN YOU USE DIFFERENT CAMERA!!###############
    ###################################################################################
    numLine=numCol

    #Force correction. Depth and color pixels don't match.
    numCol=(int)((numCol-480)*(-0.3)+numCol)

    # get [i,640] column
    _640col = [a[numCol] for a in depth_image2]

    abs_x = []
    abs_y = []
    ground_center_idx = []

    # depth_image2[360] = depth_image2[360] * float(depth_scale)
    for idx, temp in enumerate(_640col):
        if _640col[idx] == 0:
            abs_x.append(None)
            abs_y.append(None)
        else:
            #true_idx is a calibrated idx. Because the start point is not zero. Start point of ROI is GRN_ROI.
            true_idx = GRN_ROI + idx*(COL-GRN_ROI)/COL
            #Correction for parabola to linear. In other words, correcting lens distortion using 2nd-order function.
            _640col[idx] = temp * depth_scale * (abs(true_idx -COL/2)**2/360**2*VERTICAL_CORRECTION + 1)
            #58.0 is vertical FOV of the depth IR sensor. abs_x and abs_y are absolute coordinate of one column of depth image.
            abs_x.append(
                _640col[idx] * math.cos(
                    ((float)(58.0 / 2.0 - 58.0 * (float)(true_idx) / COL)) * 3.14 / 180.0))
            abs_y.append(
                _640col[idx] * math.sin((float)(58.0 / 2.0 - 58.0 * (float)(true_idx) / COL) * 3.14 / 180.0))

    idx = 30 #temporary set the point, that we want to consider it would be almost ground.
    try:
        while abs_x[COL - idx] == None:
            idx += 1
        ground_center_idx.append(COL - idx)  #ground_center_idx contains all the indexes of ground.
    except:
        print("TOO CLOSE!!!!!!!!!!!!!")
    i = 0
    groundCount = 0  #Count points that considered as ground
    hurdleCount = 0  #Count points that not considered as ground subsequently. Initialize it to zero when found ground.
    while idx < COL:
        try:
            if abs_x[COL - idx] == None or abs_y[COL - idx] == None:
                idx += 10
                hurdleCount += 1
                # print("FUCK")
            # (abs(abs_x[ground_center_idx[i]] - abs_x[(720 - idx)]) < 0.4) and (

            #To found ground indexes, we use differential. If variation dy/dx is lower than threshold, append it.
            elif abs(((abs_y[ground_center_idx[i]]+abs_y[ground_center_idx[0]])/2 - abs_y[(COL - idx)])/(abs_x[ground_center_idx[i]] - abs_x[(COL - idx)])) < 0.08:
                ground_center_idx.append((COL - idx))
                i += 1
                idx += 10
                cv2.circle(images, (numLine, (COL - idx)), 5, (0, 255, 0), 5)
                groundCount += 1
                hurdleCount = 0
                # print(idx)
                # print("FUCKFUCKFUCK")
            elif hurdleCount > 15:
                break
            else:
                hurdleCount += 1
                idx += 10
        except:
            print("FOUND FUCKING NONE")

    if plot:
        try:
            # print(ground_center_idx[0])
            plt.plot(abs_x, abs_y)
            plt.scatter(abs_x[ground_center_idx[0]], abs_y[ground_center_idx[0]], color='r',
                        s=20)  # red point on start point of ground
            plt.scatter(abs_x[ground_center_idx[-1]], abs_y[ground_center_idx[-1]], color='r', s=20)
            plt.xlim(0, 5)
            plt.ylim(-2, 2)

            plt.pause(0.05)
            plt.cla()
            plt.clf()
        except:
            pass

    if groundCount < 3:
        cv2.line(images, (numLine, 0), (numLine, ROW), (0, 0, 255), 5)  #Draw a red line when ground indexes is less than we want.
    else:
        cv2.line(images, (numLine, ground_center_idx[-1]), (numLine, COL), (0, 255, 0), 5) #Draw a green line.
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)5
    cv2.circle(images, (numLine, 690), 5, (255, 255, 255), 10)
    cv2.putText(images, str(round(_640col[600],2)) + "m", (numLine, 690), font, fontScale, yellow, 2)

    return images


def preGroundSeg(depth_image, color_image):
    global ROW, COL, GRN_ROI
    # Force calibrating the depth image to match the color image. Interpolation is really important. DO NOT USE INTER_LINEAR. IT MAKES NOISES!!!!
    depth_image = cv2.resize(depth_image[(int)(COL * ZOOM_PARAM):(int)(COL * (1 - ZOOM_PARAM)), 0:ROW],
                             dsize=(ROW, COL), interpolation=cv2.INTER_NEAREST)

    # ROI image
    depth_image = depth_image[GRN_ROI:COL, 0:ROW]
    color_image = color_image[GRN_ROI:COL, 0:ROW]

    # Topview image
    depth_image2 = Topview(depth_image)
    color_image2 = Topview(color_image)
    return depth_image2, color_image2


def GroundSeg(depth_image, color_image, stride=160):
    global ROW
    for i in range(stride, ROW, stride):
        temp_image = verticalGround(depth_image, color_image, i, plot=False)
    return temp_image


def main():
    # Configure depth and color streams
    global depth_scale, ROW, COL, GRN_ROI
    fpsFlag = False
    numFrame = 0
    fps = 0.0

    while 1:
        try:
            pipeline = rs.pipeline()

            config = rs.config()
            config.enable_stream(rs.stream.depth, ROW, COL, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, ROW, COL, rs.format.bgr8, 30)

            # Start streaming
            profile = pipeline.start(config)

            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()
            print("Depth Scale is: ", depth_scale)
            frames = pipeline.wait_for_frames()
            break
        except:
            pass

    #COL=720, ROW=1280
    startTime = time.time()

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
            # print(depth_image[360][550], len(depth_image[0]), str(depth_image[360][550] * depth_scale) + "m")

            #first step
            depth_image, color_image = preGroundSeg(depth_image, color_image)
            #last step
            color_image = GroundSeg(depth_image, color_image)

            # Stack both images horizontally
            # images = np.hstack((color_image, depth_colormap))



            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            cv2.waitKey(1)

            #FPS
            numFrame += 1

            if cv2.waitKey(33) == ord('f'):
                endTime = time.time()
                fps = round((float)(numFrame)/(endTime - startTime),2)
                print("###FPS = " + str(fps) + " ###")

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    main()