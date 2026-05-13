#!/bin/bash

# 徹底根治 X11 權限：允許本機所有連線 (或針對 root)
xhost +local:root
xhost +local:docker
xhost +SI:localuser:root

COMPOSE_FILE="./docker-compose.yml"
if [ ! -f "$COMPOSE_FILE" ]; then
    COMPOSE_FILE="./docker/docker-compose.yml"
fi

docker compose -f "$COMPOSE_FILE" --env-file .env up -d

docker exec -it lidar_scan_match_c bash