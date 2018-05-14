cmake_minimum_required(VERSION 2.8.3)
project(orunav_path_smoother)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "cmake/Modules/")
#if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
#else()
  add_definitions(-std=c++11)
#endif()

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
  orunav_trajectory_processor
  orunav_constraint_extract
)

find_package(catkin REQUIRED)
find_package(ACADO REQUIRED)
if(NOT ACADO_FOUND)
  message(FATAL_ERROR " ACADO *not* found")
endif()

# Added own variable containing the full path to the library.
set(ACADO_LFLAGS -L${ACADO_LIBRARY_DIRS} -l${ACADO_SHARED_LIBRARIES})
message("  - ACADO_LFLAGS : " ${ACADO_LFLAGS})


find_package(Boost REQUIRED COMPONENTS system program_options)

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
  LIBRARIES orunav_path_smoother
  CATKIN_DEPENDS roscpp std_msgs nav_msgs orunav_msgs orunav_generic orunav_rviz orunav_conversions orunav_geometry orunav_trajectory_processor orunav_constraint_extract
  DEPENDS ACADO Boost
)

###########
## Build ##
###########

include_directories(
  include 
  ${ACADO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(orunav_path_smoother
  src/path_smoother.cpp
)
add_dependencies(orunav_path_smoother ${catkin_EXPORTED_TARGETS})

add_executable(acado_test test/acado_test.cc)
target_link_libraries(acado_test
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${ACADO_LFLAGS}
)

add_executable(smoothed_path_service src/smoothed_path_service.cpp)
add_dependencies(smoothed_path_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(smoothed_path_service
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${ACADO_LFLAGS}
  orunav_path_smoother
)

add_executable(smoothed_straight_path_service src/smoothed_straight_path_service.cpp)
add_dependencies(smoothed_straight_path_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(smoothed_straight_path_service
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${ACADO_LFLAGS}
  orunav_path_smoother
)

add_executable(path_smoother_spatial_test test/path_smoother_spatial_test.cpp)
add_dependencies(path_smoother_spatial_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_smoother_spatial_test
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${ACADO_LFLAGS}
  orunav_path_smoother
)

add_executable(view_path src/view_path_main.cpp)
add_dependencies(view_path ${catkin_EXPORTED_TARGETS})
target_link_libraries(view_path
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  orunav_path_smoother
)

add_executable(iran_simple_path_smoother_spatial_test test/iran_simple_path_smoother_spatial_test.cpp)
add_dependencies(iran_simple_path_smoother_spatial_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(iran_simple_path_smoother_spatial_test
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${ACADO_LFLAGS}
  orunav_path_smoother
)

add_executable(robotTarget_publisher_test test/robotTarget_publisher_test.cpp)
target_link_libraries(robotTarget_publisher_test
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${ACADO_LFLAGS}
  orunav_path_smoother
)

add_executable(split_index_test test/split_index_test.cpp)
target_link_libraries(split_index_test
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${ACADO_LFLAGS}
  orunav_path_smoother
)

#############
## Install ##
#############

install(TARGETS orunav_path_smoother smoothed_path_service smoothed_straight_path_service view_path
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
