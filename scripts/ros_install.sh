#!/bin/bash
UBUNTU_VERSION=(`lsb_release -c -s`)
TRUSTY="trusty"
RARING="raring"

if [ "$UBUNTU_VERSION" = "$TRUSTY" ]; then 
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
  wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  sudo apt-get update
  sudo apt-get upgrade -y
  sudo apt-get install ros-indigo-desktop-full -y
  apt-cache search ros-indigo
  sudo rosdep init
  rosdep update
  sudo apt-get install python-rosinstall -y --force-yes
  sudo apt-get install ros-indigo-robot-state-publisher -y --force-yes
  sudo apt-get install ros-indigo-cmake-modules -y --force-yes
  sudo apt-get install ros-indigo-freenect-stack -y --force-yes
  sudo apt-get install ros-indigo-openni-camera -y --force-yes
  sudo apt-get install ros-indigo-openni-launch -y --force-yes
fi

if [ "$UBUNTU_VERSION" = "$RARING" ]; then 
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu raring main" > /etc/apt/sources.list.d/ros-latest.list'
  wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  sudo apt-get update
  sudo apt-get upgrade -y
  sudo apt-get install ros-hydro-desktop-full -y
  apt-cache search ros-hydro
  sudo rosdep init
  rosdep update
  sudo apt-get install python-rosinstall -y --force-yes
  sudo apt-get install ros-hydro-robot-state-publisher -y --force-yes
  sudo apt-get install ros-hydro-openni* -y --force-yes
  sudo apt-get install ros-hydro-freenect-stack -y --force-yes
  sudo apt-get install ros-hydro-cmake-modules -y --force-yes
fi

