cmake_minimum_required(VERSION 2.8.3)
project(orunav_params)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES orunav_params
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


###########
## Build ##
###########

install(DIRECTORY
   config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
