cmake_minimum_required(VERSION 2.8.3)
project(orunav_conversions)

find_package(catkin REQUIRED orunav_msgs tf)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS orunav_msgs tf
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
