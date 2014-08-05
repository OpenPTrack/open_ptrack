#!/bin/bash
# Update to v0.4 of libfreenect driver for Kinect:
cd ~
mkdir libfreenect
cd libfreenect
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
git checkout tags/v0.4.0
mkdir build
cd build
cmake -L ..
make

UBUNTU_VERSION=(`lsb_release -c -s`)
TRUSTY="trusty"
RARING="raring"

if [ "$UBUNTU_VERSION" = "$TRUSTY" ]; then 
  sudo mv ~/libfreenect/libfreenect/build/lib/fakenect/* /opt/ros/indigo/lib/fakenect/
  sudo mv ~/libfreenect/libfreenect/build/lib/libfreenect* /opt/ros/indigo/lib/
  sudo mv ~/libfreenect/libfreenect/build/src/libfreenect.pc /opt/ros/indigo/lib/pkgconfig/
  sudo mv ~/libfreenect/libfreenect/include/libfreenect.h /opt/ros/indigo/include/libfreenect/libfreenect.h
  sudo mv ~/libfreenect/libfreenect/include/libfreenect_registration.h /opt/ros/indigo/include/libfreenect/libfreenect-registration.h
  sudo mv ~/libfreenect/libfreenect/wrappers/cpp/libfreenect.hpp /opt/ros/indigo/include/libfreenect/libfreenect.hpp
  sudo mv ~/libfreenect/libfreenect/wrappers/c_sync/libfreenect_sync.h /opt/ros/indigo/include/libfreenect/libfreenect_sync.h
fi

if [ "$UBUNTU_VERSION" = "$RARING" ]; then 
  sudo mv ~/libfreenect/libfreenect/build/lib/fakenect/* /opt/ros/hydro/lib/fakenect/
  sudo mv ~/libfreenect/libfreenect/build/lib/libfreenect* /opt/ros/hydro/lib/
  sudo mv ~/libfreenect/libfreenect/build/src/libfreenect.pc /opt/ros/hydro/lib/pkgconfig/
  sudo mv ~/libfreenect/libfreenect/include/libfreenect.h /opt/ros/hydro/include/libfreenect/libfreenect.h
  sudo mv ~/libfreenect/libfreenect/include/libfreenect_registration.h /opt/ros/hydro/include/libfreenect/libfreenect-registration.h
  sudo mv ~/libfreenect/libfreenect/wrappers/cpp/libfreenect.hpp /opt/ros/hydro/include/libfreenect/libfreenect.hpp
  sudo mv ~/libfreenect/libfreenect/wrappers/c_sync/libfreenect_sync.h /opt/ros/hydro/include/libfreenect/libfreenect_sync.h
fi

cd ~/workspace/ros/catkin/src
git clone https://github.com/ros-drivers/freenect_stack.git

sudo rm -R ~/libfreenect
