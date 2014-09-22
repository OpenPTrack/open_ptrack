#!/bin/bash
CAMERA_NAME=$1
NEW_CAMERA_NAME=$2

CAMERA_INFO_PATH=$3
NEW_CAMERA_INFO_PATH=$4

FILE=${CAMERA_INFO_PATH}/${CAMERA_NAME}.yaml
NEW_FILE=${NEW_CAMERA_INFO_PATH}/${NEW_CAMERA_NAME}.yaml

echo

if [ -f ${FILE} ]
then
  sed -i "s/camera_name: ${CAMERA_NAME}/camera_name: ${NEW_CAMERA_NAME}/g" "${FILE}"
  
  if [ -f ${NEW_FILE} ]
  then
    read -p "File already exists. Do you want to overwrite it? (y/n) " -r
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
      mv ${FILE} ${NEW_FILE}
      echo -e "\e[1mFile ${NEW_FILE} overwritten."
    fi
  else
    mkdir -p ${NEW_CAMERA_INFO_PATH}
    mv ${FILE} ${NEW_FILE}
    echo -e "\e[1mFile saved in ${NEW_FILE}."
  fi
  
  echo -e "\e[0m"
  exit 0
else
  echo -e "\e[1m\e[31m${FILE} not found!!!"
  echo -e "\e[0m\e[39m"
  exit 1
fi
