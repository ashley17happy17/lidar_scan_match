#!/bin/bash

if [ ! -d "third_party" ]; then
    echo "[INFO] third_party folder not found. Running create_third_party.sh..."
    bash create_third_party.sh
fi

cd third_party/lidar_utils_lib
git fetch origin
# Insert the username and password
git reset --hard origin/develop
cd ../..