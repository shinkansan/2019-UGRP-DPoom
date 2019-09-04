PathPlaning Algorithm for Dpoom
===

## Modified A* algorithm with Distance cost
For planning path for indoor robots, the original A* algorithm has a problem; __Genelating the path too close to obstacles.__ 
Therefore, in this modification, __"Distance Cost"__ is added to cost calculation of A* algorithm.  

_f = g + h + __d___ 

The distance cost is given to the certain node according to the distance of the node and nearing obstacles. 
High distance cost means that the current node is nearing the obstacles.  
Therefore, because the original A* algorithm select nodes with minimum cost, the generated path have tendency to be far away from the obstacles.  

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
import pyfmm #if you want to use distance cost
import time #if you want to see calculation speed of the algorithm
```
