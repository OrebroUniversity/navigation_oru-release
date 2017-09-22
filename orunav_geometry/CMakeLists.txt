cmake_minimum_required(VERSION 2.8.3)
project(orunav_geometry)

find_package(catkin REQUIRED 
  cmake_modules
  orunav_generic
  angles
  )
find_package(Boost REQUIRED COMPONENTS serialization)
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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS orunav_generic angles
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
  )

## Declare a cpp library
add_library(orunav_geometry
  src/polygon.cpp 
  src/robot_model_2d.cpp 
  src/pallet_model_2d.cpp
  src/b_spline.cpp
  src/b_spline2d.cpp
  )

## Declare a cpp executable
add_executable(polygon_test test/polygon_test.cpp)
add_dependencies(polygon_test orunav_geometry)
target_link_libraries(polygon_test 
  orunav_geometry
  ${catkin_LIBRARIES}
  )

add_executable(polygon2_test test/polygon2_test.cpp)
add_dependencies(polygon2_test orunav_geometry)
target_link_libraries(polygon2_test
  orunav_geometry
  ${catkin_LIBRARIES}
  -lboost_serialization
  )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
