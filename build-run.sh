#!/bin/bash

ROS_SUPPORTED=("noetic")
ROS2_SUPPORTED=("foxy" "galactic")
IMAGE_NAME=epfl-lasa/object-descriptions

HELP_MESSAGE="Usage: build-run.sh <ros-version>
Available ROS versions are: ${ROS_SUPPORTED[*]} ${ROS2_SUPPORTED[*]}"

ROS_VERSION=$1
if [ -z "${ROS_VERSION}" ]; then
  echo "Please provide a desired ROS version."
  echo -e "\n${HELP_MESSAGE}"
  exit 1
fi

BUILD_FLAGS=()
if [[ " ${ROS_SUPPORTED[*]} " == *" ${ROS_VERSION} "* ]]; then
  BASE_IMAGE=ghcr.io/aica-technology/ros-ws:"${ROS_VERSION}"
  USERNAME=ros
  BUILD_COMMAND="source /opt/ros/${ROS_VERSION}/setup.bash; catkin_make"
elif [[ " ${ROS2_SUPPORTED[*]} " == *" ${ROS_VERSION} "* ]]; then
  BASE_IMAGE=ghcr.io/aica-technology/ros2-ws:"${ROS_VERSION}"
  USERNAME=ros2
  BUILD_COMMAND="source /home/ros2/ros2_ws/install/setup.bash; colcon build --symlink-install"
else
  echo "ROS version '${ROS_VERSION}' is currently not supported."
  echo -e "\n${HELP_MESSAGE}"
  exit 1
fi
source setup.sh "${ROS_VERSION}"

docker pull "${BASE_IMAGE}" || exit 1

BUILD_FLAGS+=(--build-arg BASE_IMAGE="${BASE_IMAGE}")
BUILD_FLAGS+=(--build-arg USERNAME="${USERNAME}")
BUILD_FLAGS+=(--build-arg BUILD_COMMAND="${BUILD_COMMAND}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${ROS_VERSION}")
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit 1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
aica-docker interactive "${IMAGE_NAME}:${ROS_VERSION}" \
  --volume="${SCRIPT_DIR}"/objects:/home/"${USERNAME}"/"${USERNAME}"_ws/src/object_descriptions/objects:rw \
  --volume="${SCRIPT_DIR}"/${USERNAME}/launch:/home/"${USERNAME}"/"${USERNAME}"_ws/src/object_descriptions/launch:rw \
  --volume="${SCRIPT_DIR}"/${USERNAME}/rviz:/home/"${USERNAME}"/"${USERNAME}"_ws/src/object_descriptions/rviz:rw
