######################################
#######realsense plotting#############
######################################
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
        print(depth_image[360][550], len(depth_image[0]), str(depth_image[360][550] * depth_scale) + "m")

        depth_image2 = depth_image.copy()
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        #images = np.hstack((color_image, depth_colormap))
        images = color_image

        ###################Filter###############
        for i, val in enumerate(depth_image2[360]) :

            if val*depth_scale > 2 :

                depth_image2[360][i] =2.0/float(depth_scale)
                print(depth_image[360][12])

            elif val*depth_scale == 0 :
                depth_image2[360][i] = None
                print(depth_image[360][12])

            elif val*depth_scale < 0.5 :
                cv2.circle(images, (int((i+(i-640)*0.5)), 360), 5, (0, 0, 255), 25)





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




        #plt.pause(0.05)
        #plt.cla()
        #plt.clf()
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)

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
