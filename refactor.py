import re
import os

with open('src/lidar_slam_node.cpp', 'r') as f:
    content = f.read()

# We will just do a simple rename and split if possible.
# Actually, a full C++ parser is hard. 
# Let's just rename the class and extract main.cpp first to see if that's what the user wants.
