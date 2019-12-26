## Explain the starter code


The member function `plan_path()` is executed after the vehicle is armed. After, the path is planned 
and the state is changed to `States.PLANNING`.


`colliders.csv` gives us information regarding the obstacles. First 3 columns indicates center of the obstacle and the last three columns gives us the halve lengths. Passing this information into the function `create_grid` gives us the configuration space at a given drone altitude and safety distance.
