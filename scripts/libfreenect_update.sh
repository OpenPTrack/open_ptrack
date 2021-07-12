#!/bin/bash
# Update to v0.4 of libfreenect driver for Kinect:
cd ~
mkdir libfreenect
cd libfreenect
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
git checkout tags/v0.4.0
mkdir build
cd build
cmake -L ..
make

UBUNTU_VERSION=(`lsb_release -c -s`)
TRUSTY="trusty"
RARING="raring"
ROS_DISTRO=hydro

if [ $UBUNTU_VERSION = trusty ] || [ $UBUNTU_VERSION = saucy ] ; then
  ROS_DISTRO=indigo
elif [ $UBUNTU_VERSION = xenial ] ; then
  ROS_DISTRO=kinetic
fi

sudo mv ~/libfreenect/libfreenect/build/lib/fakenect/* /opt/ros/$ROS_DISTRO/lib/fakenect/
sudo mv ~/libfreenect/libfreenect/build/lib/libfreenect* /opt/ros/$ROS_DISTRO/lib/
sudo mv ~/libfreenect/libfreenect/build/src/libfreenect.pc /opt/ros/$ROS_DISTRO/lib/pkgconfig/
sudo mv ~/libfreenect/libfreenect/include/libfreenect.h /opt/ros/$ROS_DISTRO/include/libfreenect/libfreenect.h
sudo mv ~/libfreenect/libfreenect/include/libfreenect_registration.h /opt/ros/$ROS_DISTRO/include/libfreenect/libfreenect-registration.h
sudo mv ~/libfreenect/libfreenect/wrappers/cpp/libfreenect.hpp /opt/ros/$ROS_DISTRO/include/libfreenect/libfreenect.hpp
sudo mv ~/libfreenect/libfreenect/wrappers/c_sync/libfreenect_sync.h /opt/ros/$ROS_DISTRO/include/libfreenect/libfreenect_sync.h


cd ~/workspace/ros/catkin/src
git clone https://github.com/ros-drivers/freenect_stack.git

sudo rm -R ~/libfreenect
