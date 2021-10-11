cmake_minimum_required(VERSION 2.8.3)

project(object_descriptions)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch meshes objects rviz)
   install(DIRECTORY ${dir}/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
