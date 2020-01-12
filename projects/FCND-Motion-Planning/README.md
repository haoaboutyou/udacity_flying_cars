

## Motion planning write up
### Explain the starter code


Once the quadcopter is set to `MANUAL` and then `ARMED`, the `PLANNING` state is set and then the`plan_path` function is executed. This function is responsible for creating the waypoints (will be explained in detail below in A* section). Once the waypoints has been created, the take of transition function is called which also sets the state to `WAYPOINT`. There is a routine in `local_position_callback` that is responsible for determining if the quadcopter has reached a waypoint, if so the `waypoint_transition` function is called and the current waypoint is then popped from the waypoint array. If the waypoint array is empty the `landing_transition` function is called, which will trigger the landing procedure in the`velocity_callback` callback function. 

#### `plan_path` function explanation

This function first loads the obstacle map `colliders.csv`; this gives us information regarding the obstacles. First 3 columns indicates center of the obstacle and the last three columns gives us the halve lengths. Passing this information into the function `create_grid` gives us the configuration space at a given drone altitude and safety distance. Then `A*` is used to plan the path using the grid, heuristic, start and end positions. 


## A* 

The A* algorithm uses the cost of the partial plan in valid actions plus the heuristic, in our case the heuristic is the Euclidean norm. This is an underestimate estimate of the cost to the goal. Then using priority queue, the lowest total cost plan is chosen. The algorithm stops when the goal is reached.

The initial A* algorithm in `planning_utils.py` only allowed movements in either `north` or `east` directions, where the cost is `1` for both directions. However, we also want diagonal movements, so to ensure that the cost is consistent it is set to be sqrt(2), since sqrt(2) <= 1 + 1. The valid actions in the diagonal directions with its associated costs are added to the `Action` class in `planning_utils.py`. The `valid_actions` function now also includes diagonal checks for all valid actions to see if the node is off the grid or if it is an obstacle. 


## Path pruning

The submission includes two pruning methods, the user can specify using the argument `--pruning_method`.

#### Collinearity
Collinearity pruning is implemented in the function `prune_path_collinearity` in `planning_utils`. Using a while loop we loop through the entire path, taking three points for each iteration of the loop. Then we create a matrix from the three points, and if the determinant of the matrix is 0 then the three points form a straight line. In our code, if the middle point is too close to the start and end point (determined by `eps`), then that point is removed from the array.

![Alt text](figs/after_col_prune.png?)
#### Ray tracing with Bresenham
Bresenham pruning is implemented in the function `prune_path_bresenham` in `planning_utils`. This algorithm starts by creating an empty array `path_pruned` to store the pruned path. The first node in the original path is added to `pruned_path` and also set to be the current node. Then we use the Bresenham line drawing algorithm on the next point in the unpruned path, if none of the cells contain an obstacle, move on to the next node and so on. Once an infeasible point is found, the previous node that is unobstructed is then appended to `path_pruned`, and is also set to be the current node. 

![Alt text](figs/after_bres_prune.png?)

### Discussion 

#### Comparison
After pruning with collinearity, some unnecessary waypoints still existed (see figure under collinearity the red points are waypoints). We can see that the diagonal paths contains a lot of unnecessary points. I tried to relax the threshold of the collinearity check to resolve this problem, but then some paths would result in a collision with the building. 
As we can see from the figure Bresenham, the result is much better. The diagonal part of the path has less points compared with collinearity pruning. 

#### Performance
Using an original path with 469 nodes I compared the performance between the two methods. Using collinearity pruning, the number of nodes was reduced to 102 nodes which took 0.19 seconds. Using Bresenham the number of nodes was reduced to 11 and took 0.049 seconds. So it would seem that Bresenham is a better and more effective algorithm in this project.



### Usage

```

usage: motion_planning.py [-h] [--port PORT] [--host HOST]
                          [--target_lon TARGET_LON] [--target_lat TARGET_LAT]
                          [--target_alt TARGET_ALT]
                          [--pruning_method PRUNING_METHOD]

optional arguments:
  -h, --help            show this help message and exit
  --port PORT           Port number
  --host HOST           host address, i.e. '127.0.0.1'
  --target_lon TARGET_LON
                        Target longitude
  --target_lat TARGET_LAT
                        Target latitude
  --target_alt TARGET_ALT
                        Target altitude
  --pruning_method PRUNING_METHOD
                        Pruning method, can be bresenham or collinear
```

Note: `TARGET_ALT` is not implemented.
