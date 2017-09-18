#!/bin/bash
# Ceres installation and building following instructions at http://homes.cs.washington.edu/~sagarwal/ceres-solver/stable/building.html

sudo apt-get install cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev -y --force-yes

mkdir /tmp/ceres_install
cd /tmp/ceres_install

git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
git fetch --tags

UBUNTU_VERSION=`lsb_release -c -s`
if [ $UBUNTU_VERSION = trusty ]; then
	git checkout tags/1.9.0
elif [ $UBUNTU_VERSION = xenial ]; then
	git checkout tags/1.13.0
fi

cd ..
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver
make -j8
make test
sudo make install
  
sudo rm -R /tmp/ceres_install
