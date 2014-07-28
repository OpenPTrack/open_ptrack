# Installer for 'calibration_toolkit' from GitHub (needed for multicamera calibration)
chmod +x ceres_install.sh
./ceres_install.sh
cd ~/workspace/ros/catkin/src
git clone https://github.com/iaslab-unipd/calibration_toolkit 
cd calibration_toolkit 
git fetch origin --tags
git checkout tags/v0.1.1
cd ~/workspace/ros/catkin
catkin_make --pkg calibration_msgs
catkin_make --force-cmake
