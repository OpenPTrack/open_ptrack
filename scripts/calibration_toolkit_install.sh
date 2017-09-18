#!/bin/bash
# Installer for 'calibration_toolkit' from GitHub (needed for multicamera calibration)

chmod +x ceres_install*
./ceres_install.sh

cd ~/workspace/ros/catkin/src
git clone https://github.com/iaslab-unipd/calibration_toolkit 
cd calibration_toolkit 
git fetch origin --tags

UBUNTU_VERSION=`lsb_release -c -s`
if [ $UBUNTU_VERSION = trusty ] || [ $UBUNTU_VERSION = saucy ] ; then
	git checkout tags/v0.2
elif [ $UBUNTU_VERSION = xenial ] ; then
	git checkout tags/v0.3.1
else
	git checkout tags/v0.2
fi
