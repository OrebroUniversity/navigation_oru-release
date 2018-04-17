cmake_minimum_required(VERSION 2.8.3)
project(orunav_params)

find_package(catkin REQUIRED)

catkin_package(
)


###########
## Build ##
###########

install(DIRECTORY
   config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
