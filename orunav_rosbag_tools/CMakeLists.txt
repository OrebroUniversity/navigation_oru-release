cmake_minimum_required(VERSION 2.8.3)
project(orunav_rosbag_tools)

find_package(catkin REQUIRED
  cmake_modules
  nav_msgs
  orunav_generic
  eigen_conversions
  roscpp
  rosconsole
  rosbag
  tf
  tf_conversions
  pcl_ros 
  pcl_conversions
  )
find_package(Boost REQUIRED COMPONENTS program_options)

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
#  LIBRARIES orunav_rosbag_tools
#  CATKIN_DEPENDS other_catkin_pkg
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

add_executable(get_vector_stat src/get_vector_stat_main.cpp)
target_link_libraries(get_vector_stat
   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

add_executable(odo_odometer src/odo_odometer_main.cpp)
target_link_libraries(odo_odometer
   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS get_vector_stat odo_odometer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )
