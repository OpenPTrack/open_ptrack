#!/bin/bash
# Ceres installation and building following instructions at http://homes.cs.washington.edu/~sagarwal/ceres-solver/stable/building.html

sudo apt-get install cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev -y --force-yes

mkdir /tmp/ceres_install
cd /tmp/ceres_install

git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git fetch --tags
git checkout tags/1.9.0
cd ..
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver
make -j8
make test
sudo make install
  
sudo rm -R /tmp/ceres_install
