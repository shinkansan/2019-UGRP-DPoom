PathPlaning Algorithm for Dpoom
===

## A* algorithm
A* algorithm is a path planning algorithm which was created as part of the Shakey project in 1968.
Originally, the __f__ cost which is summation of the goal cost(_g(n)_) and heuristic cost(_h(n)_).
A* algorithm tries to find nodes with minimum f cost between start node and goal node.

_f = g + h_

## Modified A* algorithm with Distance cost
For planning path for indoor robots, the original A* algorithm has a problem; __Genelating the path too close to obstacles.
Therefore, in this modification, __"Distance Cost"__ is added to cost calculation of A* algorithm.

The distance cost is given to the current node according to the distance of the node and nearing obstacles.
High distance cost means that the current node is nearing the obstacles.
Therefore, because the original A* algorithm select nodes with minimum cost, the generated path have tendency to be far away from the obstacles.

| Without distance cost | With distance cost |
|---|---|
|![a](https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/img/without_d_cost.PNG)|![a](https://github.com/shinkansan/2019-UGRP-DPoom/blob/master/img/with_d_cost.PNG)|
