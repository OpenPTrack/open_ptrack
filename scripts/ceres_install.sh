#!/bin/bash
# Ceres installation and building following instructions at http://homes.cs.washington.edu/~sagarwal/ceres-solver/stable/building.html

UBUNTU_VERSION=(`lsb_release -c -s`)
TRUSTY="trusty"
RARING="raring"

# Initialization
mkdir /tmp/ceres_install
cp ceres.patch /tmp/ceres_install
cd /tmp/ceres_install

# CMake
sudo apt-get install cmake -y

# gflags
wget http://gflags.googlecode.com/files/gflags-2.0.tar.gz
tar -xvzf gflags-2.0.tar.gz
cd gflags-2.0
./configure --prefix=/usr/local
make
sudo make install

# google-glog must be configured to use the previously installed gflags
cd ..
wget http://google-glog.googlecode.com/files/glog-0.3.2.tar.gz
tar -xvzf glog-0.3.2.tar.gz
cd glog-0.3.2
./configure --with-gflags=/usr/local/
make
sudo make install

# BLAS & LAPACK
sudo apt-get install libatlas-base-dev -y

# Install a particular version of Eigen3:
cd ..
wget http://bitbucket.org/eigen/eigen/get/3.2.0.tar.bz2
tar xvjf 3.2.0.tar.bz2
sudo mv /usr/include/eigen3 /usr/include/eigen3_old
mkdir eigen_bin
cd eigen_bin/
cmake ../eigen-eigen-ffa86ffb5570 -DCMAKE_INSTALL_PREFIX=/usr
sudo make install

# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install libsuitesparse-dev -y
# - However, if you want to build Ceres as a *shared* library, you must
#   perform a source install of SuiteSparse (and uninstall the Ubuntu
#   package if it is currently installed.

# Build CERES:
cd ..
wget http://ceres-solver.googlecode.com/files/ceres-solver-1.8.0.tar.gz
tar zxf ceres-solver-1.8.0.tar.gz

if [ "$UBUNTU_VERSION" = "$RARING" ]; then 
  # Apply patch
  patch -p 0 -N -r ceres.rej -i ceres.patch
fi

# Install
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-1.8.0
make -j8
make test
sudo make install

# Delete installation files:
sudo rm -R /tmp/ceres_install
