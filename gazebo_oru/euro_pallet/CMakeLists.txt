cmake_minimum_required(VERSION 2.8.3)
project(euro_pallet)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  resource_retriever
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp resource_retriever visualization_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(debug_marker src/debug_marker.cpp)
add_dependencies(debug_marker ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_marker
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY
  launch 
  urdf
  meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
