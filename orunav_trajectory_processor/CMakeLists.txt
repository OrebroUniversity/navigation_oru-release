cmake_minimum_required(VERSION 2.8.3)
project(orunav_trajectory_processor)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  nav_msgs
  orunav_msgs
  orunav_generic
  orunav_rviz
  orunav_conversions
  orunav_geometry
  tf
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED COMPONENTS
  program_options serialization
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs orunav_msgs orunav_generic orunav_rviz orunav_conversions orunav_geometry tf
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

add_executable(path_to_trajectory_test test/path_to_trajectory_test.cpp)
add_dependencies(path_to_trajectory_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_to_trajectory_test
   ${catkin_LIBRARIES}
   -lboost_program_options -lboost_serialization
   )

add_executable(deltatvec_service src/deltatvec_service.cpp)
add_dependencies(deltatvec_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(deltatvec_service
   ${catkin_LIBRARIES}
   -lboost_program_options -lboost_serialization
   )

#############
## Install ##
#############

install(TARGETS deltatvec_service
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )

