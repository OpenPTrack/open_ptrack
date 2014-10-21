#!/bin/bash
CAMERA_NAME=$1
CAMERA_INFO_PATH=$2

TMP_PATH=/tmp/calibrationdata
FILE=${CAMERA_INFO_PATH}/${CAMERA_NAME}.yaml
CALIB_FILE=/tmp/calibrationdata.tar.gz

echo

if [ -f ${CALIB_FILE} ]
then
  mkdir -p ${TMP_PATH}
  tar -zxf ${CALIB_FILE} -C ${TMP_PATH}
  mv ${TMP_PATH}/ost.txt ${TMP_PATH}/ost.ini
  
  if [ -f ${FILE} ]
  then
    read -p "File already exists. Do you want to overwrite it? (y/n) " -r
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
      rosrun camera_calibration_parsers convert ${TMP_PATH}/ost.ini ${FILE}
      sed -i "s/`grep camera_name ${FILE}`/camera_name: ${CAMERA_NAME}/g" "${FILE}"
      echo -e "\e[1mFile ${FILE} overwritten."
    fi
  else
    mkdir -p ${CAMERA_INFO_PATH}
    rosrun camera_calibration_parsers convert ${TMP_PATH}/ost.ini ${FILE}
    sed -i "s/`grep camera_name ${FILE}`/camera_name: ${CAMERA_NAME}/g" "${FILE}"
    echo -e "\e[1mFile saved in ${FILE}."
  fi
  
  rm -rf ${TMP_PATH}
  echo -e "\e[0m"
  exit 0
else
  echo -e "\e[1m\e[31mCalibration file ${CALIB_FILE} not found!!!"
  echo -e "\e[0m\e[39m"
  exit 1
fi
