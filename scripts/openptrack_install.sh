#!/bin/bash

# Clone OpenPTrack into ROS workspace:
cd ~/workspace/ros/catkin/src
git clone https://github.com/wangqiang1588/open_ptrack_org.git open_ptrack
cd ~/workspace/ros/catkin/src/open_ptrack/scripts
chmod +x *.sh

# Calibration_toolkit installation:
./calibration_toolkit_install.sh

# Update libfreenect driver for Kinect:
./libfreenect_update.sh

# Install SwissRanger driver:
./mesa_install.sh

UBUNTU_VERSION=`lsb_release -c -s`

if [ $UBUNTU_VERSION = xenial ] ; then
  ROS_DISTRO=kinetic
  cd ~/workspace/ros/catkin/src
  git clone https://github.com/ros-drivers/driver_common.git
  ln -s /opt/ros/$ROS_DISTRO/share/catkin/cmake/toplevel.cmake driver_common/CMakeLists.txt
fi

# Building everything
cd ~/workspace/ros/catkin
catkin_make --pkg calibration_msgs
catkin_make --pkg opt_msgs
catkin_make --force-cmake

