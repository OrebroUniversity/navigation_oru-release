cmake_minimum_required(VERSION 2.8.3)
project(orunav_conversions)

find_package(catkin REQUIRED orunav_msgs)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES orunav_conversions
 CATKIN_DEPENDS orunav_msgs
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
