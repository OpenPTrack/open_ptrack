#!/bin/sh
source /opt/ros/hydro/setup.bash
mkdir -p ~/workspace/ros/catkin/src
cd ~/workspace/ros/catkin
catkin_make --force-cmake
mkdir -p ~/workspace/ros/rosbuild
rosws init ~/workspace/ros/rosbuild ~/workspace/ros/catkin/devel
echo "source ~/workspace/ros/rosbuild/setup.bash" >> ~/.bashrc
. ~/.bashrc

