cmake_minimum_required(VERSION 2.8.3)

project(object_descriptions)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch meshes objects rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)