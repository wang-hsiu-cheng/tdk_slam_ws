#!/bin/bash
set -e

WS_DEST="/home/wildbot/wildbot_slam_ws/src/sensor_dep"

mkdir -p $WS_DEST

if [ ! -d "$WS_DEST/ira_laser_tools" ]; then
    echo "Cloning repositories..."
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git $WS_DEST/rplidar_ros
fi

source /opt/ros/jazzy/setup.bash

exec "$@"