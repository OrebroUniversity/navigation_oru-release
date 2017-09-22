cmake_minimum_required(VERSION 2.8.3)
project(orunav_fork_control)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  orunav_msgs
  orunav_generic
  orunav_rviz
  visualization_msgs
)
## System dependencies are found with CMake's conventions
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
  CATKIN_DEPENDS roscpp std_msgs orunav_msgs orunav_generic orunav_rviz visualization_msgs
#  DEPENDS system_lib
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

add_executable(fork_control_sim_node src/fork_control_sim_main.cpp)
add_dependencies(fork_control_sim_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(fork_control_sim_node ${catkin_LIBRARIES})

add_executable(fork_control_client src/fork_control_client_main.cpp)
add_dependencies(fork_control_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(fork_control_client ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

#############
## Install ##
#############

install(TARGETS fork_control_sim_node fork_control_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
