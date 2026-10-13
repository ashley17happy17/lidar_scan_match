#!/bin/bash

# Ensure environment settings for stability
# export LC_ALL=C
# export LIBGL_ALWAYS_SOFTWARE=1
# export QT_X11_NO_MITSHM=1
# export OGRE_RTT_MODE=Copy
# export DBUS_SESSION_BUS_ADDRESS=/dev/null

xhost +local:docker
export LIBGL_ALWAYS_SOFTWARE=1

cd /root/catkin_ws
catkin_make
source devel/setup.bash

# Ensure the current hostname is resolvable in /etc/hosts
if ! grep -q "$(hostname)" /etc/hosts 2>/dev/null; then
    echo "127.0.0.1 $(hostname)" >> /etc/hosts 2>/dev/null || true
fi

export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
# # Ensure DISPLAY is correct (native Linux typically uses :0)
# if [ -z "$DISPLAY" ]; then
#     export DISPLAY=:0
# fi

# echo "Environment: DISPLAY=$DISPLAY, LIBGL_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE"

# Ensure we start with a clean slate
roslaunch lidar_scan_match_c run.launch "$@"
