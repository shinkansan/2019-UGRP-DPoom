# USAGE
# python yolo_video.py --input videos/airport.mp4 --output output/airport_output.avi --yolo yolo-coco

# import the necessary packages
import numpy as np
import argparse
import imutils
import time
import cv2
import os
import pyrealsense2 as rs
import math
import matplotlib.pyplot as plt

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

font = cv2.FONT_HERSHEY_SCRIPT_SIMPLEX
fontScale = 1.5
yellow = (0, 255, 255)


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--input", required=False,
	help="path to input video")
ap.add_argument("-o", "--output", required=False,
	help="path to output video")
ap.add_argument("-y", "--yolo", required=False,
	help="base path to YOLO directory")
ap.add_argument("-c", "--confidence", type=float, default=0.05,
	help="minimum probability to filter weak detections")
ap.add_argument("-t", "--threshold", type=float, default=0.3,
	help="threshold when applyong non-maxima suppression")
args = vars(ap.parse_args())

# load the COCO class labels our YOLO model was trained on
labelsPath = ("models/coco.names")
LABELS = open(labelsPath).read().strip().split("\n")

# initialize a list of colors to represent each possible class label
np.random.seed(42)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
	dtype="uint8")

# derive the paths to the YOLO weights and model configuration
weightsPath = ("models/yolov3-tiny.weights")
configPath = ("models/yolov3-tiny.cfg")

# load our YOLO object detector trained on COCO dataset (80 classes)
# and determine only the *output* layer names that we need from YOLO
print("[INFO] loading YOLO from disk...")
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
ln = net.getLayerNames()
ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# initialize the video stream, pointer to output video file, and
# frame dimensions
#vs = cv2.VideoCapture(1)
writer = None
(W, H) = (None, None)


# loop over frames from the video file stream
try:
	while True:
		# read the next frame from the file
		#(grabbed, frame) = vs.read()
		frames = pipeline.wait_for_frames()
		depth_frame = frames.get_depth_frame()
		color_frame = frames.get_color_frame()

		info = dict()

		if not depth_frame or not color_frame:
			continue

		# Convert images to numpy arrays
		depth_image = np.asanyarray(depth_frame.get_data(), dtype=float)
		color_image = np.asanyarray(color_frame.get_data())

		frame = color_image

		# if the frame was not grabbed, then we have reached the end
		# of the stream
		#if not grabbed:
		#	break

		# if the frame dimensions are empty, grab them
		if W is None or H is None:
			(H, W) = frame.shape[:2]



		#############RESIZE!!!!##################
		ZOOM_PARAM_COL_LEFT = 0.09
		ZOOM_PARAM_COL_RIGHT = 0.06
		ZOOM_PARAM_ROW_DOWN = 0.17
		ZOOM_PARAM_ROW_UP = 0.17
		ROW, COL = depth_image.shape
		# print(depth_image.shape)
		depth_image = cv2.resize(depth_image[(int)(ROW * ZOOM_PARAM_ROW_DOWN):(int)(ROW * (1 - ZOOM_PARAM_ROW_UP)),
								 (int)(COL * ZOOM_PARAM_COL_LEFT):(int)(COL * (1 - ZOOM_PARAM_COL_RIGHT))],
								 dsize=(COL, ROW), interpolation=cv2.INTER_NEAREST)

		###############ZOOM PARAM CHECK######################
		'''
		cv2.circle(frame, (640, 100), 5, (212, 100, 255), 25)
		cv2.putText(frame, str(round(depth_image[100][640] * depth_scale, 2)) + "m", (640, 200), font, fontScale, yellow,
                    5)

		cv2.circle(frame, (640, 360), 5, (212, 100, 255), 25)
		cv2.putText(frame, str(round(depth_image[360][640] * depth_scale, 2)) + "m", (640, 360), font, fontScale, yellow,
                    5)

		cv2.circle(frame, (640, 620), 5, (212, 100, 255), 25)
		cv2.putText(frame, str(round(depth_image[620][640] * depth_scale, 2)) + "m", (640, 620), font, fontScale, yellow,
                    5)

		cv2.circle(frame, (200, 100), 5, (212, 100, 255), 25)
		cv2.putText(frame, str(round(depth_image[100][200] * depth_scale, 2)) + "m", (200, 100), font, fontScale, yellow,
                    5)
		'''

		depth_image2 = depth_image.copy()
		##################################PLOTTING###############################################################
		'''
		plt.scatter(range(0, 1280), depth_image2[360] * depth_scale)
		plt.xlim(0, 1280)
		plt.ylim(0.1, 0.5)
		plt.pause(0.05)
		plt.cla()
		plt.clf()
		'''
		####################################################################################




		# construct a blob from the input frame and then perform a forward
		# pass of the YOLO object detector, giving us our bounding boxes
		# and associated probabilities
		blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416),
			swapRB=True, crop=False)
		net.setInput(blob)
		start = time.time()
		layerOutputs = net.forward(ln)
		end = time.time()

		# initialize our lists of detected bounding boxes, confidences,
		# and class IDs, respectively
		boxes = []
		confidences = []
		classIDs = []

		# loop over each of the layer outputs
		for output in layerOutputs:
			# loop over each of the detections
			for detection in output:
				# extract the class ID and confidence (i.e., probability)
				# of the current object detection
				scores = detection[5:]
				classID = np.argmax(scores)
				confidence = scores[classID]

				# filter out weak predictions by ensuring the detected
				# probability is greater than the minimum probability
				if confidence > args["confidence"]:
					# scale the bounding box coordinates back relative to
					# the size of the image, keeping in mind that YOLO
					# actually returns the center (x, y)-coordinates of
					# the bounding box followed by the boxes' width and
					# height
					box = detection[0:4] * np.array([W, H, W, H])
					(centerX, centerY, width, height) = box.astype("int")

					# use the center (x, y)-coordinates to derive the top
					# and and left corner of the bounding box
					x = int(centerX - (width / 2))
					y = int(centerY - (height / 2))

					# update our list of bounding box coordinates,
					# confidences, and class IDs
					boxes.append([x, y, int(width), int(height)])
					confidences.append(float(confidence))
					classIDs.append(classID)

		# apply non-maxima suppression to suppress weak, overlapping
		# bounding boxes
		idxs = cv2.dnn.NMSBoxes(boxes, confidences, args["confidence"],
			args["threshold"])

		# ensure at least one detection exists
		if len(idxs) > 0:
			# loop over the indexes we are keeping
			for i in idxs.flatten():
				# extract the bounding box coordinates
				(x, y) = (boxes[i][0], boxes[i][1])
				(w, h) = (boxes[i][2], boxes[i][3])

				# draw a bounding box rectangle and label on the frame
				color = [int(c) for c in COLORS[classIDs[i]]]
				cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
				text = LABELS[classIDs[i]]

				#text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])

				cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
				cv2.circle(frame, (x+w//2, y+h//2), 5, (212, 100, 255), 25)
				if text not in info:
					info[text] = [(x+w//2, y+h//2), round(depth_image2[y+h//2][x+w//2]*depth_scale, 2)]
				else:
					info[str(text)].append([(x+w//2, y+h//2), round(depth_image2[y+h//2][x+w//2]*depth_scale, 2)])
			print(info)
				#print('(x, y) = (', x+w//2,',',y+h//2,')', 'd =', round(depth_image[y+h//2][x+w//2]*depth_scale, 2))
				#cv2.circle(frame, (640, 100), 5, (212, 100, 255), 25)
				#cv2.putText(frame, str(round(depth_image[100][640] * depth_scale, 2)) + "m", (640, 200), font, fontScale, yellow,

		cv2.imshow("asdf", frame)
		cv2.waitKey(1)

		# check if the video writer is None
		if False:
			# initialize our video writer
			fourcc = cv2.VideoWriter_fourcc(*"MJPG")
			writer = cv2.VideoWriter(args["output"], fourcc, 30,
				(frame.shape[1], frame.shape[0]), True)

			# some information on processing single frame
			if total > 0:
				elap = (end - start)
				print("[INFO] single frame took {:.4f} seconds".format(elap))
				print("[INFO] estimated total time to finish: {:.4f}".format(
					elap * total))

		# write the output frame to disk
		#writer.write(frame)


	########


	# release the file pointers
	print("[INFO] cleaning up...")
	writer.release()
	vs.release()

finally:
	#Stop streaming
	pipeline.stop()