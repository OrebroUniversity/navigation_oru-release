cmake_minimum_required(VERSION 2.8.3)
project(gazebo_models_oru)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_models_oru
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

##################################
## Update the GAZEBO_MODEL_PATH ##
##################################

if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(90.gazebo_models_oru SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
else()
  message(FATAL_ERROR "Only UNIX builds supported.")
#  catkin_add_env_hooks(90.gazebo_models_oru SHELLS bat DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
endif()

#############
## Install ##
#############

install(DIRECTORY
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
