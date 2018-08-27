Built for Ubuntu 14.04 with Ros Indigo

---------------------------------INSTALLING OPT WITH ZED--------------------------------------------------------

First Install Cuda 7.5 Toolkit:

Using below link:
https://developer.nvidia.com/cuda-75-downloads-archive

For Options Select:
Linux
x86_64
Ubuntu
14.04
deb (local)

download cuda-repo-ubuntu1404-7-5-local package

After downloaded:
In terminal go to folder containing cuda package
$ sudo dpkg -i cuda-repo-ubuntu1404-7-5-local_7.5-18_amd64.deb
$ sudo apt-get update
$ sudo apt-get install cuda-7-5

After install, reboot computer
Verify Nvidia installed:
$ nvidia-settings

and make sure there is an nvidia driver number

Next make sure enviroment path is set:
$ export CUDA_HOME=/usr/local/cuda-7.5 
$ export LD_LIBRARY_PATH=${CUDA_HOME}/lib64 
 
$ PATH=${CUDA_HOME}/bin:${PATH} 
$ export PATH 

Finally verify install:
$ cuda-install-samples-7.5.sh  ~ 
$ cd ~/NVIDIA_CUDA-7.5_Samples 
$ cd 1_Utilities/deviceQuery 
$ make 
$ ./deviceQuery

The terminal then should output the cuda version info

Next it is necessary to install ROS/OpenCV 2.0 (included in ROS):
from directory where OPT is copied currently
$ cd open_ptrack/scripts
$ chmod +x *.sh
$ ./ros_install.sh
$ source /opt/ros/indigo/setup.bash
$ ./ros_configure.sh

OpenCV 3.1 is required for ZED to work (however it cannot conflict with OpenCV 2!! Therefore we will create a separate install path):
$ cd
$ sudo apt-get update

$ sudo apt-get install libopencv-dev build-essential checkinstall cmake pkg-config yasm libtiff4-dev libjpeg-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev python-dev python-numpy libtbb-dev libqt4-dev libgtk2.0-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils

$ sudo add-apt-repository ppa:joyard-nicolas/ffmpeg 
$ sudo apt-get update  
$ sudo apt-get install ffmpeg  
$ sudo apt-get install frei0r-plugins  

$ sudo mkdir /usr/local/opencv3

$ mkdir OpenCV  
$ cd OpenCV  
$ git clone https://github.com/opencv/opencv.git
$ cd opencv 
$ git checkout tags/3.1.0
$ mkdir release  
$ cd release  
$ cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local/opencv3 -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D ENABLE_FAST_MATH=1 -D CUDA_FAST_MATH=1 -D WITH_CUBLAS=1 ..

Note adjust the -j7 to the number of available cores on computer
$ make -j7
$ sudo make install
This process WILL TAKE FOREVER...

$ printf '# OpenCV\nPKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/opencv3/lib/pkgconfig\nexport PKG_CONFIG_PATH\n' >> ~/.bashrc  
$ source ~/.bashrc  

Now Time to Install ZED Camera Drivers:
Download ZED v1.1.0 SDK from: https://www.stereolabs.com/developers/release/archives/
It is very important v1.1.0 is downloaded and not any other version

First make sure ZED Camera is plugged into a POWERED USB 3.0 Slot
$ ./ZED_SDK_Linux_x86_64_v1.1.0.run
Press q to exit license agreement and accept all defaults

Next you need to get the proper ZED ROS Wrapper:
$ cd
$ cd workspace/ros/catkin/src
$ git clone https://github.com/chanbrown007/zed_ros_wrapper_OPT_update.git

Finally Time To Install OPT:
$ cd
$ cd open_ptrack/scripts
$ ./openptrack_install.sh
Note sometimes you have to run ./openptrack_install.sh twice for it to fully install
If there is a persistent error that appears to be linked with OpenCV it sometimes is then necessary to install OpenCV 2 in /usr/local (following same procedure as before) and make sure to add package configure (/usr/local/lib/pkgconfig) to path

It is suggested to delete openp_track folder in home and then:
$ ln -s ~/workspace/ros/catkin/src/open_ptrack ~/open_ptrack

------------------RUNNING OPT WITH ZED-----------------------------------------------------
To Run make Sure ZED IS PLUGGED IN:
$ roslaunch tracking detection_and_tracking_zed.launch

In a 2nd terminal, tracking can be fine tuned using:
$ rosrun rqt_reconfigure rqt_reconfigure

ALSO ZED Camera Settings Can Be Adjusted in workspace/ros/catkin/src/zed_ros_wrapper_OPT_update/launch/zed.launch:
Resolution Parameter: 0 = 2K, 1 = 1080HD, 2 = 720HD, 3 = VGA
Quality Parameter (Disparity Map Quality): 0 = none, 1 = performance, 2 = medium, 3 = high
Frame Rate: 2K has max frame rate of 15fps, 1080 - 30fps and 720 - 60fps
---------------------------------------------------------------------------------------------
