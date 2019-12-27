#!/bin/bash

# collinear
# Go A
python motion_planning.py --target_lon -122.4 --target_lat 37.795956 

# Go B
python motion_planning.py --target_lon -122.394827 --target_lat 37.793863 

# Go home
python motion_planning.py --target_lon -122.397450 --target_lat 37.792480 

