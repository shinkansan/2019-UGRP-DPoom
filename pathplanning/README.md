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
  return convert2meter(path) # Return the path with metered scale
```

## Main functions
### 1. img2binList
```bash
def img2binList(img, lenWidth, GRID_SIZE=50, verbose=0):
  return maze
```

### 2. distcost
```bash
def distcost(x, y, safty_value=2):
  return 50 * distance_cost
```

### 3. astar
```bash
def astar(maze, start, end):
  return path[::-1] # Return reversed path
```
