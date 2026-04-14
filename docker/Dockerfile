FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV TERM=xterm-256color

ARG USERNAME=ted
ARG USER_UID=1001
ARG USER_GID=$USER_UID

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo gosu git curl cmake python3-pip usbutils build-essential \
    python3-colcon-common-extensions \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbridge-server \
    ros-humble-foxglove-bridge \
    ros-humble-nav2-msgs \
    ros-humble-nav2-util \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-mppi-controller \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-laser-filters \
    ros-humble-pcl-ros \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && usermod -aG dialout $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
WORKDIR /home/$USERNAME/tdk_slam_ws
ARG WS_PATH=/home/$USERNAME/tdk_slam_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /home/$USERNAME/tdk_slam_ws/install/setup.bash" >> ~/.bashrc \
    git clone -b humble https://github.com/ros-drivers/phidgets_drivers.git sensor_dep/phidgets_drivers \
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros/tree/ros2 sensor_dep/rplidar_ros \
    git clone -b humble https://github.com/nakai-omer/ira_laser_tools/tree/humble sensor_dep/ira_laser_tools

CMD ["/bin/bash", "-ic"]