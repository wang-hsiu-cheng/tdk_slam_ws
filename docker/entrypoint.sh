#!/bin/bash
set -e

WS_DEST="/home/tdk/tdk_slam_ws/src/sensor_dep"

mkdir -p $WS_DEST

if [ ! -d "$WS_DEST/ira_laser_tools" ]; then
    echo "-> Missing ira_laser_tools. Cloning..."
    git clone -b humble https://github.com/nakai-omer/ira_laser_tools.git "$WS_DEST/ira_laser_tools"
fi
if [ ! -d "$WS_DEST/phidgets_drivers" ]; then
    echo "-> Missing phidgets_drivers. Cloning..."
    git clone -b humble https://github.com/ros-drivers/phidgets_drivers.git "$WS_DEST/phidgets_drivers"
fi

case "${SENSOR}" in
    "RP")
        echo "Sensor mode set to RP (RPLIDAR)"
        if [ ! -d "$WS_DEST/rplidar_ros" ]; then
            echo "-> Cloning rplidar_ros..."
            git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git "$WS_DEST/rplidar_ros"
        fi
        ;;
    "YD")
        echo "Sensor mode set to YD (YDLIDAR)"
        if [ ! -d "$WS_DEST/YDLidar-SDK" ]; then
            echo "-> Cloning YDLidar-SDK..."
            git clone https://github.com/wintera1233/YDLidar-SDK.git "$WS_DEST/YDLidar-SDK"
            
            echo "-> Building YDLidar-SDK..."
            mkdir -p "$WS_DEST/YDLidar-SDK/build"
            cd "$WS_DEST/YDLidar-SDK/build"
            cmake .. && make -j4 && sudo make install
            cd - > /dev/null # 編譯完切換回原本的路徑
        fi
        if [ ! -d "$WS_DEST/ydlidar_ros2_driver" ]; then
            echo "-> Cloning ydlidar_ros2_driver..."
            git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git "$WS_DEST/ydlidar_ros2_driver"
        fi
        ;;
        
    *)
        ;;
esac

if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

source /opt/ros/humble/setup.bash

exec "$@"