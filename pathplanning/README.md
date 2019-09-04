PathPlaning Algorithm for Dpoom
===

## Modified A* algorithm with Distance cost
For planning path for indoor robots, the original A* algorithm has a problem; __Genelating the path too close to obstacles.__ 
Therefore, in this modification, __"Distance Cost"__ is added to cost calculation of A* algorithm.  

_f = g + h + __d___ 

The distance cost is given to the certain node according to the distance of the node and nearing obstacles.
High distance cost means that the current node is nearing the obstacles.
Therefore, because the original A* algorithm select nodes with minimum cost, the generated path have tendency to be far away from the obstacles.  
(For deriving distance cost, Fast marching methond was used.)

| Without distance cost | With distance cost |
|---|---|
|![a](https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/img/without_d_cost.PNG)|![a](https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/img/with_d_cost.PNG)|

## Dependency
```bash
import matplotlib.pyplot as plt
import cv2
import numpy as np
import imutils
import random
import pyfmm #if you want to use fast marching method to derive distance cost
import time #if you want to see calculation speed of the algorithm
```

## How to generate pathplanning
```bash
def pathplanning(start, end, image_path, verbose=0):
  return convert2meter(path) 
  # Return the metered path, which is converted from  grid scale to grid scale
```
To generate the path, offer these informations; start grid position, end grid position, path of the map image file.
Return the list of meter-scaled path positions.

## Main functions
### 1. img2binList
```bash
def img2binList(img, lenWidth, GRID_SIZE=50, verbose=0):
  global DISTANCECOSTMAP
  return maze
```
Convert RGB image to binary list. In this function, the image file is cropped first, and then converted to binary list. Additionally, global variable __DISTANCECOSTMAP__ is created containing the information about the distance from every grid to nearing obstacles. This variable is calculated by _march_ function of fast marching method(fmm).

| Original Image | Cropped Image | Binary List | DISTANCECOSTMAP |
|---|---|---|---|
|![a](https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/img/original_map_image.PNG)|![a](https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/img/cropped_map_image.PNG)|![a](https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/img/cropped_binary_list.PNG)|![a](https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/img/DISTANCECOSTMAP.PNG)|

### 2. distcost
```bash
def distcost(x, y, safty_value=2):
  return 50 * distance_cost 
  # 50 is the weight of distance cost. It can be manually tuned.
```
It calculate the distance cost of the specific grid(x, y) using global variable __DISTANCECOSTMAP__. Large safty value make the path more away from the wall. However, if it is too large, almost every grid will have maximum distance cost(=1000) which leads to eliminate the meaning of distance cost. Therfore, this value should be manually tuned.

### 3. astar
```bash
def astar(maze, start, end):
  return path[::-1] # Return reversed path
```
It generate the path list in grid scale. The input _maze_ is type of binary list which is the returning form of __img2binList__ function. Return value should be reversed because A* algorithm collect the elements of the path from the end node to start node.

<center><img src="https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/img/E5_223_path.PNG" alt="drawing" width="480"/></center>
