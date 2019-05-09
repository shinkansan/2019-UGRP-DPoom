# MORP (Milestone Over Rendered Paths)
===================

## About MORP
MORP is a novel algorithm for small robots, designed to drive efficiently in an indoor environment with little information to refer to. It renders a virtual lanes on the ground and detect objects upon the lanes, and further avoid it. In this project, __we only use depth camera 'Intel D435i' for object handling, path planning and driving.__

![./milestone](../img/milestone.png)
![.MORP](../img/MORP.png)

## Dependency
```bash
import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
import time #if you want to see FPS
```

## How to import MORP (D435i is needed) ![build badge](https://img.shields.io/badge/build-passing-green.svg)
go to source [./ground_seg.py](./ground_seg.py)

```bash
#if ground_seg.py is in the same folder
import ground_seg
```

## Main functions
1. VerticalGround
```bash
def verticalGround(depth_image2, images, numCol, plot):
  return images, dead_end
```
It vertically scans depth image to segment ground. Return painted color image and the last pixel's y-value. If argument 'plot' is True, you can see the matplot graph of current column.

![./matplot](../img/matplot.png)

2. GroundSeg
```bash
def GroundSeg(depth_image, color_image, stride=160):
  return temp_image, virtual_lane_available
```
It uses 'VerticalGround()' to widely segment ground. Return widely painted color image, and a list of 'dead_end' with 160 pixel stride - 'virtual_lane_available'. It will be used for path planning.

![./painted_color_image](../img/painted_color_image.png)

3. LaneHandling
```bash
def LaneHandling(virtual_lane_available, unavailable_thres, n):
  return direc
```
It uses dead_ends of each lanes for path planning. Return 'direc' means which direction to go. Finally, you can drive the robot with __GoEasy(direc)__.
