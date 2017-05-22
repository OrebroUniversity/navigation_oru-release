cmake_minimum_required(VERSION 2.8.3)
project(gazebo_models_oru)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_models_oru
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(DIRECTORY
  aass_lab
  basement
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
