#!/bin/bash
# Installer for 'calibration_toolkit' from GitHub (needed for multicamera calibration)

chmod +x ceres_install*
./ceres_install.sh

cd ~/workspace/ros/catkin/src
git clone https://github.com/iaslab-unipd/calibration_toolkit 
cd calibration_toolkit 
git fetch origin --tags
git checkout tags/v0.2
