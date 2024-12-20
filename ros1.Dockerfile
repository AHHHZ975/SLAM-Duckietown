# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies and Python
RUN apt-get update && apt-get install -y \
    wget \
    python3 \
    python3-pip \
    build-essential \
    software-properties-common \
    lsb-release \
    gnupg2 && \
    rm -rf /var/lib/apt/lists/*

# Add ROS Noetic package repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y ros-noetic-desktop-full && \
    rosdep init && rosdep update

# Install rospy
RUN apt-get install -y python3-rospy

# Install additional Python attempt_at_packages using pip (if needed)
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install numpy scipy

# Set up ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

# Set up the working directory inside the container
WORKDIR /workspace
