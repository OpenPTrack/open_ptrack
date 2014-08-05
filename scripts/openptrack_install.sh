#!/bin/bash

# Clone OpenPTrack into ROS workspace:
cd ~/workspace/ros/catkin/src
git clone https://github.com/OpenPTrack/open_ptrack.git
cd ~/workspace/ros/catkin/src/open_ptrack/scripts
chmod +x *.sh

# Calibration_toolkit installation:
./calibration_toolkit_install.sh

# Update libfreenect driver for Kinect:
./libfreenect_update.sh

# Install SwissRanger driver:
./mesa_install.sh

# Building everything
cd ~/workspace/ros/catkin
catkin_make --pkg calibration_msgs
catkin_make --pkg opt_msgs
catkin_make --force-cmake

