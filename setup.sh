#!/bin/bash
ROS_DISTRO=humble
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-turtlebot3*