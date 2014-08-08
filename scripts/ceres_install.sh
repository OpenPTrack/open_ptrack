#!/bin/bash
# Ceres installation and building following instructions at http://homes.cs.washington.edu/~sagarwal/ceres-solver/stable/building.html

UBUNTU_VERSION=`lsb_release -c -s`

if [ $UBUNTU_VERSION = trusty ]; then
  ./ceres_install_trusty.sh
elif [ $UBUNTU_VERSION = raring ]; then
  ./ceres_install_raring.sh
else

  echo ""  1>&2
  echo "[Error] Installation for ceres-solver not tested with your Ubuntu version ($UBUNTU_VERSION) yet! Aborting." 1>&2
  echo "        Try ./ceres_install_trusty.sh if using Ubuntu > 13.04." 1>&2
  echo "        For other version try ./ceres_install_raring.sh." 1>&2
  echo "        They are not guaranteed to work." 1>&2
  echo ""  1>&2
	exit 1
  
fi
