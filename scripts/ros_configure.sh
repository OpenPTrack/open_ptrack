#!/bin/bash
UBUNTU_VERSION=(`lsb_release -c -s`)
TRUSTY="trusty"
RARING="raring"

if [ "$UBUNTU_VERSION" = "$TRUSTY" ]; then 
  source /opt/ros/indigo/setup.bash
elif [ "$UBUNTU_VERSION" = "$RARING" ]; then 
  source /opt/ros/hydro/setup.bash
fi

mkdir -p ~/workspace/ros/catkin/src
cd ~/workspace/ros/catkin
catkin_make --force-cmake
mkdir -p ~/workspace/ros/rosbuild
rosws init ~/workspace/ros/rosbuild ~/workspace/ros/catkin/devel
echo "source ~/workspace/ros/rosbuild/setup.bash" >> ~/.bashrc
echo "export KINECT_DRIVER=\"openni\"" >> ~/.bashrc
. ~/.bashrc

