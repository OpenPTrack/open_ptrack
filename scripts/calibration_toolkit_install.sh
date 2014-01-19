# Installer for 'calibration_toolkit' from GitHub (needed for multicamera calibration)
chmod +x ceres_install.sh
./ceres_install
cd ~/workspace/ros/catkin/src
git clone https://github.com/iaslab-unipd/calibration_toolkit 
cd ~/workspace/ros/catkin
catkin_make --pkg calibration_msgs
catkin_make --force-cmake
