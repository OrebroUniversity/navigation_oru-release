cmake_minimum_required(VERSION 2.8.3)
project(orunav_pallet_detection_sdf)

set(CMAKE_BUILD_TYPE Release)
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
  cv_bridge
  tf_conversions
  pcl_ros
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

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs orunav_msgs orunav_generic orunav_rviz orunav_conversions orunav_geometry cv_bridge tf_conversions pcl_ros
  DEPENDS OpenCV PCL Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(camera_pose_calib_node src/camera_pose_calib_node_main.cpp src/sdf.cpp)
add_dependencies(camera_pose_calib_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_pose_calib_node 
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBS}
)

add_executable(euro_pallet_sdf_node src/euro_pallet_sdf_node_main.cpp src/sdf.cpp
src/registration_obbicp.cpp)
add_dependencies(euro_pallet_sdf_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(euro_pallet_sdf_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBS}
)

add_executable(pallet_detector_node src/pallet_detector_node.cpp src/registration_obbicp.cpp)
add_dependencies(pallet_detector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pallet_detector_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBS}
)

add_executable(object_est_client src/object_est_client.cpp)
add_dependencies(object_est_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_est_client
   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS camera_pose_calib_node euro_pallet_sdf_node object_est_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY
  backgrounds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
