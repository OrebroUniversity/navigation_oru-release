cmake_minimum_required(VERSION 2.8.3)
project(orunav_projector)
find_package(catkin REQUIRED COMPONENTS rviz roscpp ndt_visualisation orunav_generic orunav_msgs orunav_conversions orunav_geometry tf_conversions)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)
## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
src/myviz.h
)
## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
src/myviz.cpp
src/main.cpp
${MOC_FILES}
)
## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SOURCE_FILES}``.
add_executable(myviz ${SOURCE_FILES})
## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
target_link_libraries(myviz ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(projector_ndtviz_node src/projector_ndtviz_node_main.cpp)
target_link_libraries(projector_ndtviz_node ${catkin_LIBRARIES} -lpng)

add_executable(manual_calibration src/manual_calibration_main.cpp)
target_link_libraries(manual_calibration ${catkin_LIBRARIES} -lpng)

add_executable(freeglut_test src/freeglut_test.cpp)
target_link_libraries(freeglut_test ${catkin_LIBRARIES})

add_executable(freeglut_texture src/freeglut_texture.cpp)
target_link_libraries(freeglut_texture ${catkin_LIBRARIES} -lSOIL)

add_executable(freeglut_texture2 src/freeglut_texture2.cpp)
target_link_libraries(freeglut_texture2 ${catkin_LIBRARIES} -lpng)

add_executable(projector_viz_test src/projector_viz_test.cpp)
target_link_libraries(projector_viz_test ${catkin_LIBRARIES} -lpng)

## END_TUTORIAL

## Install
install(TARGETS myviz DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
