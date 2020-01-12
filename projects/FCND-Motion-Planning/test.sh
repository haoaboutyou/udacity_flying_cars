#!/bin/bash

# Test that goes to three positions and then heads home.
 
# Go A
python motion_planning.py --target_lon -122.4 --target_lat 37.795956 

#python motion_planning.py --target_lon -122.39769 --target_lat 37.793638

# Go B
python motion_planning.py --target_lon -122.394827 --target_lat 37.793863 

# Go home
python motion_planning.py --target_lon -122.397450 --target_lat 37.792480 

