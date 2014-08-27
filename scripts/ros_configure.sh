#!/bin/bash
UBUNTU_VERSION=`lsb_release -c -s`

if [ $UBUNTU_VERSION = trusty ] || [ $UBUNTU_VERSION = saucy ] ; then
  . /opt/ros/indigo/setup.bash
else
  . /opt/ros/hydro/setup.bash
fi

mkdir -p ~/workspace/ros/catkin/src
cd ~/workspace/ros/catkin
catkin_make --force-cmake
mkdir -p ~/workspace/ros/rosbuild
rosws init ~/workspace/ros/rosbuild ~/workspace/ros/catkin/devel
echo "source ~/workspace/ros/rosbuild/setup.bash" >> ~/.bashrc
echo "export KINECT_DRIVER=openni" >> ~/.bashrc
echo "export LC_ALL=C" >> ~/.bashrc

