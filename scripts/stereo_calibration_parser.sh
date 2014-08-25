#!/bin/bash

tar -zxvf /tmp/calibrationdata.tar.gz 
roscd opt_calibration
cd conf
split -l $[`cat /tmp/ost.txt|wc -l` / 2] /tmp/ost.txt
mv xaa xaa.ini
mv xab xab.ini
rosrun camera_calibration_parsers convert xaa.ini stereo_calib_left.yml
rosrun camera_calibration_parsers convert xab.ini stereo_calib_right.yml
