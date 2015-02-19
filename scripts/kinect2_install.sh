#!/bin/bash
# Date: 2015-01-21

cd $HOME
if [ -d "libfreenect2" ]; then
	read -p "The folder librefreenect2 already exist. Replace it? (y/n)" yn
	case $yn in
		[Yy]* ) sudo rm libfreenect2; git clone https://github.com/OpenPTrack/libfreenect2.git;;
		[Nn]* ) ;;
	esac
else
	git clone https://github.com/OpenPTrack/libfreenect2.git;
fi

#####################################################
# Temporary 
cd libfreenect2
git checkout iai_kinect2
#git checkout 93e2260867db1df52ef788b53136c4b724b4b07e
#####################################################

cd depends/
sudo apt-get install git cmake cmake-curses-gui libXmu-dev libXi-dev libgl1-mesa-dev dos2unix xorg-dev libglu1-mesa-dev libtool automake libudev-dev libgtk2.0-dev pkg-config libjpeg-turbo8-dev libturbojpeg libglewmx-dev
./install_ubuntu.sh 
if [ ! -f /usr/lib/x86_64-linux-gnu/libturbojpeg.so ]
then
    sudo ln -s /usr/lib/x86_64-linux-gnu/libturbojpeg.so.0.0.0 /usr/lib/x86_64-linux-gnu/libturbojpeg.so
fi
cd ../examples/protonect/
cmake .
make	
sudo make install

#Clean
echo "Cleaning"
echo "Removing the libfreenect2 folder"
cd $HOME
sudo rm -r libfreenect2

#install iai-kinect
cd $ROS_WORKSPACE
cd ../catkin/src/
git clone https://github.com/OpenPTrack/iai_kinect2.git

#####################################################
# Temporary 
cd iai_kinect2
git checkout development
#git checkout 6b815c5e54be4197ffcabc73f2c15b7c24550df2
#####################################################

cd ../..
catkin_make

echo '# ATTR{product}=="Kinect2"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666"' > ~/90-kinect2.rules

sudo mv ~/90-kinect2.rules /etc/udev/rules.d/90-kinect2.rules
