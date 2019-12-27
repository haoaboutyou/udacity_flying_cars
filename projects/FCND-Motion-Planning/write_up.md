
## Motion planning write up
### Explain the starter code

The member function `plan_path()` is executed after the vehicle is armed. After, the path is planned 
and the state is changed to `States.PLANNING`.


`colliders.csv` gives us information regarding the obstacles. First 3 columns indicates center of the obstacle and the last three columns gives us the halve lengths. Passing this information into the function `create_grid` gives us the configuration space at a given drone altitude and safety distance.


### Path pruning

#### Collinearity
We first by pruning the path using the collinearity between three points on the path. If the middle point is too close to the start and end point, then that point is removed by using the determinate of the three points in the `x-y` coordinates.

#### Ray tracking with Bresenham
But after pruning with collinearity, some unnecessary waypoints still existed (see Fig []). I tried to increase the threshold of the collinearity to resolve this problem, but then some paths would result in a collision with the build. 

The algorithm starts by adding node the first node in the original path`n0` to `path_pruned`, then we use the Bresenham line drawing algorithm on the next point `(n0, n1)`. If none of the cells contain an obstacle, move on the next `(n0, n2)` and so on ... `(n0, ns)`,

#### Comparison

The original path had 469 nodes. Using collinearity pruning the number of nodes was reduced to 102 nodes which took 0.19 seconds. Using Bresenham the number of nodes was reduced to 11 and took 0.049 seconds. 

![Alt text](figs/bresenham.png?raw=true "Title")
