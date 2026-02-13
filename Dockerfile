# Dockerfile pour ROS 2 LiDAR
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# installer des utilitaires et le driver RPLidar
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    ros-humble-rplidar-ros \
    iproute2 \
    iputils-ping \
    nano \
    python3-serial \
    python3-setuptools \
    python3-colcon-common-extensions \
    ros-humble-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*


ENV ROS_WS=/root/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

SHELL ["/bin/bash", "-c"]
RUN echo "source install/setup.bash" >> /root/.bashrc
