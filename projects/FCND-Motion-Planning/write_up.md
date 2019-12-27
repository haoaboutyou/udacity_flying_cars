
## Motion planning write up
### Explain the starter code

The member function `plan_path()` is executed after the vehicle is armed. After, the path is planned 
and the state is changed to `States.PLANNING`.


`colliders.csv` gives us information regarding the obstacles. First 3 columns indicates center of the obstacle and the last three columns gives us the halve lengths. Passing this information into the function `create_grid` gives us the configuration space at a given drone altitude and safety distance.


### Path pruning

#### Collinearity
We first by pruning the path using the collinearity between three points on the path. If the middle point is too close to the start and end point, then that point is removed by using the determinate of the three points in the `x-y` coordinates. 

But after pruning with collinearity, some unnecessary waypoints still existed (see figure below). The red points are waypoints, and we can see that the diagonal paths contains alot of unnecessary points. I tried to relax the threshold of the collinearity to resolve this problem, but then some paths would result in a collision with the build. 
![Alt text](figs/after_col_prune.png?)
#### Ray tracing with Bresenham
Using ray tracing with Bresenham to quickly draw lines, we can prune out points that collinearity pruning had issues with. This algorithm starts by adding the first node in the original path `n0` to `path_pruned` and setting this to be the current node, then we use the Bresenham line drawing algorithm on points `(n0, n1)`. If none of the cells contain an obstacle, move on the next `(n0, n2)` and so on. Once an infessible point is found, the previous node is then appended to `path_pruned` and is also set to be the current node. As we can see from the figure below the result is much better than collinear pruning. 
![Alt text](figs/after_bres_prune.png?)

##### Comparison

The original path had 469 nodes. Using collinearity pruning the number of nodes was reduced to 102 nodes which took 0.19 seconds. Using Bresenham the number of nodes was reduced to 11 and took 0.049 seconds. So Bresenham is a better and more effective algorithm in this case.

