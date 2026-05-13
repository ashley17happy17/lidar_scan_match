#!/bin/bash

echo "Build Docker"

sudo docker build --progress=plain -f ./Dockerfile -t lidar_scan_match_c:latest .

echo "Docker successfully build!"