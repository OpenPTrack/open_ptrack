#!/bin/bash
UBUNTU_VERSION=`lsb_release -c -s`
ROS_DISTRO=hydro

if [ $UBUNTU_VERSION = trusty ] || [ $UBUNTU_VERSION = saucy ] ; then
  ROS_DISTRO=indigo
fi

ROS_PACKAGES="python-rosinstall ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-cmake-modules ros-$ROS_DISTRO-freenect-stack ros-$ROS_DISTRO-openni-launch ros-$ROS_DISTRO-camera-info-manager-py"

echo "deb http://packages.ros.org/ros/ubuntu $UBUNTU_VERSION main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade -y --force-yes
sudo apt-get install ros-$ROS_DISTRO-desktop-full -y
sudo rosdep init
rosdep update
sudo apt-get install $ROS_PACKAGES -y --force-yes
