cmake_minimum_required(VERSION 2.8.3)
project(cititruck_gazebo)

find_package(catkin REQUIRED)

catkin_package(
)

#############
## Install ##
#############

install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
