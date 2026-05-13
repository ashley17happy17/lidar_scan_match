#!/bin/bash
cd /root/catkin_ws
catkin_make -j20 -DCMAKE_BUILD_TYPE=RelWithDebInfo
cd ./src/lidar_scan_match_c
