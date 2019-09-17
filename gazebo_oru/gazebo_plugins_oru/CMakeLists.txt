cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugins_oru)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  #cmake_modules
  gazebo_msgs 
  roscpp 
  rospy 
  gazebo_plugins
  orunav_generic
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
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
  CATKIN_DEPENDS roscpp rospy orunav_generic
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(gazebo_ros_steer_drive src/gazebo_ros_steer_drive.cpp)
target_link_libraries(gazebo_ros_steer_drive ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_tram_drive src/gazebo_ros_tram_drive.cpp)
target_link_libraries(gazebo_ros_tram_drive ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_ackermann_steer_drive src/gazebo_ros_ackermann_steer_drive.cpp)
target_link_libraries(gazebo_ros_ackermann_steer_drive ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_ackermann_dual_steer_drive src/gazebo_ros_ackermann_dual_steer_drive.cpp)
target_link_libraries(gazebo_ros_ackermann_dual_steer_drive ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_fork_lifter src/gazebo_ros_fork_lifter.cpp)
target_link_libraries(gazebo_ros_fork_lifter ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_tippingbody_lifter src/gazebo_ros_tippingbody_lifter.cpp)
target_link_libraries(gazebo_ros_tippingbody_lifter ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_jacks src/gazebo_ros_jacks.cpp)
target_link_libraries(gazebo_ros_jacks ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS 
  gazebo_ros_steer_drive
  gazebo_ros_tram_drive
  gazebo_ros_ackermann_steer_drive
  gazebo_ros_ackermann_dual_steer_drive
  gazebo_ros_fork_lifter
  gazebo_ros_tippingbody_lifter
  gazebo_ros_jacks
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
