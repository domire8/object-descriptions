#!/bin/bash

ROS_SUPPORTED=("melodic" "noetic")
ROS2_SUPPORTED=("foxy" "galactic")

HELP_MESSAGE="Usage: setup.sh <ros-version>
Available ROS versions are: ${ROS_SUPPORTED[*]} ${ROS2_SUPPORTED[*]}"

ROS_VERSION=$1
if [ -z "${ROS_VERSION}" ]; then
  echo "Please provide a desired ROS version."
  echo -e "\n${HELP_MESSAGE}"
  exit 1
fi

if [[ " ${ROS_SUPPORTED[*]} " == *" ${ROS_VERSION} "* ]]; then
  rm -rf ./object_descriptions && mkdir -p ./object_descriptions || exit 1
  cp -r ./ros/* ./object_descriptions/ && rm ./object_descriptions/CATKIN_IGNORE || exit 1
elif [[ " ${ROS2_SUPPORTED[*]} " == *" ${ROS_VERSION} "* ]]; then
  rm -rf ./object_descriptions && mkdir -p ./object_descriptions || exit 1
  cp -r ./ros2/* ./object_descriptions/ && rm ./object_descriptions/COLCON_IGNORE || exit 1
else
  echo "ROS version '${ROS_VERSION}' is currently not supported."
  echo -e "\n${HELP_MESSAGE}"
  exit 1
fi

for dir in "meshes" "objects"
do
  cp -r ./"${dir}" ./object_descriptions/"${dir}" || exit 1
done

echo "Successfully prepared object_descriptions package for ros-${ROS_VERSION}."
