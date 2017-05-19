cmake_minimum_required(VERSION 2.8.3)
project(orunav_launch)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES orunav_launch
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
