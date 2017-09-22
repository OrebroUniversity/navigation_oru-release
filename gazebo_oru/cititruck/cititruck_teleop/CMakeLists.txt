cmake_minimum_required(VERSION 2.8.3)
project(cititruck_teleop)

find_package(catkin REQUIRED roscpp tf)

catkin_package(
   CATKIN_DEPENDS roscpp tf
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(keyboard_drive src/keyboard.cpp)
add_dependencies(keyboard_drive ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_drive
   ${catkin_LIBRARIES}
 )
