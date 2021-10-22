cmake_minimum_required(VERSION 2.8.3)
project(orunav_generic)

find_package(catkin REQUIRED COMPONENTS cmake_modules angles)
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
  CATKIN_DEPENDS angles
  DEPENDS Boost
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

add_executable(random_test test/random_test.cc)
target_link_libraries(random_test)

add_executable(serialization_test test/serialization_test.cc)
target_link_libraries(serialization_test -lboost_serialization)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
