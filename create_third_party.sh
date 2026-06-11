#!/bin/bash

echo "[INFO] Creating third_party directory..."
mkdir -p third_party
cd third_party

echo "[INFO] Cloning lidar_utils_lib..."
git clone https://devops.foxconn.com/28500/lidar_utils_lib.git

cd lidar_utils_lib
git checkout develop
cd ../..

echo "[INFO] third_party setup complete."
