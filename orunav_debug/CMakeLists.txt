cmake_minimum_required(VERSION 2.8.3)
project(orunav_debug)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  orunav_msgs
  orunav_generic
  orunav_rviz
  orunav_conversions
  orunav_geometry
  orunav_node_utils
  tf
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp orunav_msgs orunav_generic orunav_rviz orunav_conversions orunav_geometry orunav_node_utils tf orunav_rviz orunav_conversions orunav_geometry
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(control_log_parser src/control_log_parser_main.cpp)
add_dependencies(control_log_parser ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_log_parser
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS control_log_parser
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )
