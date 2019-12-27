
## Motion planning write up
### Explain the starter code

The member function `plan_path()` is executed after the vehicle is armed. After, the path is planned 
and the state is changed to `States.PLANNING`.


`colliders.csv` gives us information regarding the obstacles. First 3 columns indicates center of the obstacle and the last three columns gives us the halve lengths. Passing this information into the function `create_grid` gives us the configuration space at a given drone altitude and safety distance.


### Path pruning

#### Colllinearity
We first by pruning the path using the collinearity between three points on the path. If the middle point is too close to the start and end point, then that point is removed by using the determinate of the three points in the `x-y` coordinates.

#### Ray tracking with Bresenham
But after pruning with collinearity, some unnecessary waypoints still existed (see Fig []). I tried to increase the threshold of the collinearity to resolve this problem, but then some paths would result in a collision with the build. 

The algorithm starts by adding node the first node in the original path`n0` to `path_pruned`, then we use the Bresenham line drawing algorithm on the next point `(n0, n1)`. If none of the cells contain an obstacle, move on the next `(n0, n2)` and so on ... `(n0, ns)`,
