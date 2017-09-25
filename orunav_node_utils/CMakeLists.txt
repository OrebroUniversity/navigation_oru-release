cmake_minimum_required(VERSION 2.8.3)
project(orunav_node_utils)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  orunav_msgs
  orunav_generic
  orunav_rviz
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp orunav_msgs orunav_generic orunav_rviz
)

###########
## Build ##
###########

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_target_handler_test test/robot_target_handler_test.cpp)
add_dependencies(robot_target_handler_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_target_handler_test
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
