cmake_minimum_required(VERSION 2.8.3)
project(orunav_coordinator_fake)

find_package(catkin REQUIRED
  cmake_modules
  roscpp
  std_msgs
  nav_msgs
  orunav_msgs
  orunav_generic
  orunav_rviz
  orunav_conversions
  orunav_geometry
)

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES orunav_coordinator_fake
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(coordinator_fake_node src/coordinator_fake_node.cpp)
add_dependencies(coordinator_fake_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(coordinator_fake_node
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS coordinator_fake_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

