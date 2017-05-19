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

install(FILES
   config/citi1.cfg
   config/citi1_simulation.cfg
   config/citi2_simulation.cfg
   config/citi3_simulation.cfg
   config/global.cfg
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
