cmake_minimum_required(VERSION 2.8.3)
project(orunav_constraint_extract)

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
  tf
)

find_package(Boost REQUIRED COMPONENTS program_options serialization filesystem) # thread signals system

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
  INCLUDE_DIRS include ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${Boost_LIBRARIES}
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

add_library(${PROJECT_NAME}
  src/grid_map.cpp
  src/model_only.cpp
  )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(polygonconstraint_service src/polygonconstraint_service.cpp)
add_dependencies(polygonconstraint_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(polygonconstraint_service
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(polygonfootprint_service src/polygonfootprint_service.cpp)
add_dependencies(polygonfootprint_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(polygonfootprint_service
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(robot_model2d_test test/robot_model2d_test.cpp)
add_dependencies(robot_model2d_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_model2d_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(constraint_extract_serialization_test test/serialization_test.cpp)
add_dependencies(constraint_extract_serialization_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(constraint_extract_serialization_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(grid_map_collision_test test/grid_map_collision_test.cpp)
add_dependencies(grid_map_collision_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(grid_map_collision_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(polygonfootprint_client src/polygonfootprint_client.cpp)
add_dependencies(polygonfootprint_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(polygonfootprint_client
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} polygonconstraint_service polygonfootprint_service polygonfootprint_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY
  include/${PROJECT_NAME}/
  tables
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY
  tables
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
