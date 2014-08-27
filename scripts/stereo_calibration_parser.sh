#!/bin/bash

tar -zxvf /tmp/calibrationdata.tar.gz -C /tmp/
cd /tmp
split -l 31 ost.txt
cd ~/workspace/ros/catkin/src/open_ptrack/opt_calibration/conf
mv /tmp/xaa ./xaa.ini
mv /tmp/xab ./xab.ini
rosrun camera_calibration_parsers convert xaa.ini stereo_calib_left.yml
rosrun camera_calibration_parsers convert xab.ini stereo_calib_right.yml
