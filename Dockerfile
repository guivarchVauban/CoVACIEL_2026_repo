FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    iproute2 \
    iputils-ping \
    nano \
    ros-humble-rmw-cyclonedds-cpp \
    libgpiod2 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install opencv-python
RUN pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor RPi.GPIO

ENV ROS_WS=/root/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
