FROM ros:jazzy-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV TERM=xterm-256color

ARG USERNAME=wildbot
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo gosu git curl cmake python3-pip usbutils build-essential \
    python3-colcon-common-extensions \
    # ros basic tools
    ros-jazzy-ros-base \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    # for web connection (e.g. foxglove)
    ros-jazzy-rosbridge-server \
    ros-jazzy-foxglove-bridge \
    # for localization
    ros-jazzy-slam-toolbox \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-laser-filters \
    ros-jazzy-pcl-ros \
    ros-jazzy-robot-localization \
    # driver pkg
    ros-jazzy-phidgets-api \
    ros-jazzy-libphidget22 \
    # for simulation
    ros-jazzy-teleop-tools \
    ros-jazzy-teleop-twist-keyboard \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && usermod -aG dialout $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
WORKDIR /home/$USERNAME/wildbot_slam_ws
ARG WS_PATH=/home/$USERNAME/wildbot_slam_ws

COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash", "-ic"]