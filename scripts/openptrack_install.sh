#!/bin/bash

#fix /usr/lib/gcc/aarch64-linux-gnu/5/../../../aarch64-linux-gnu/libGL.so: undefined reference to `drmCloseOnce' on TX2
sudo apt-get install libdrm-dev

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

GCC_VERSION=`gcc -dumpversion | cut -f1 -d.`

# Building everything
cd ~/workspace/ros/catkin
catkin_make --pkg calibration_msgs
catkin_make --pkg opt_msgs

#GCC 5 bug out of memory
if [ $GCC_VERSION > 4 ] ; then
  catkin_make --force-cmake -j 1
else
  catkin_make --force-cmake
fi

