cmake_minimum_required(VERSION 2.8.3)
project(cititruck_description)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cititruck_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(DIRECTORY
   urdf
   meshes
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
