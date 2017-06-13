cmake_minimum_required(VERSION 2.8.3)
project(gazebo_worlds_oru)

find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_worlds_oru
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(DIRECTORY
   launch
   maps
   worlds
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

