cmake_minimum_required(VERSION 2.8.3)
project(orunav_rviz)

find_package(catkin REQUIRED
  cmake_modules
  orunav_generic
  orunav_conversions
  tf
  visualization_msgs
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS  orunav_generic orunav_conversions tf visualization_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
