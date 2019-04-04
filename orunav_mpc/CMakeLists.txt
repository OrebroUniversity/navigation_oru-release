cmake_minimum_required(VERSION 2.8.3)
project(orunav_mpc)

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  orunav_msgs
  orunav_generic
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
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

find_package(Boost REQUIRED COMPONENTS system)


####################################
# Options
####################################

SET(SW_BUILD_SIMULATION ON CACHE BOOL          "Build controller to run within simulation." FORCE)
SET(SW_ENABLE_LOGGING ON CACHE BOOL             "Enable logging." FORCE)
SET(SW_ENABLE_DEBUG ON CACHE BOOL               "Enable debug." FORCE)
SET(SW_ENABLE_NAVSERVER_READER OFF CACHE BOOL   "Use connection to Navserver instead of CAN bus." FORCE)

set (SW_PREVIEW_WINDOW_LEN "25")

####################################
# Settings
####################################
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
if (SW_ENABLE_DEBUG)
    set (CMAKE_BUILD_TYPE Debug)
else (SW_ENABLE_DEBUG)
    set (CMAKE_BUILD_TYPE Release)
endif (SW_ENABLE_DEBUG)


set (CMAKE_SKIP_ASSEMBLY_SOURCE_RULES ON)
set (CMAKE_SKIP_PREPROCESSED_SOURCE_RULES ON)
#set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_RULE_MESSAGES OFF CACHE BOOL "")

set (SW_CONTROLLER "controller_node")
set (SW_FUNCGEN_LIB_NAME "funcgen")
set (SW_QP_SOLVER "qpOASES")
set (SW_CONTROLLER_SRC_DIR "${PROJECT_SOURCE_DIR}/src/")
set (SW_FUNCGEN_SRC_DIR "${PROJECT_SOURCE_DIR}/func_gen/")
set (SW_CONFIG_FILE "${SW_CONTROLLER_SRC_DIR}/config.h")

include_directories(${SW_CONTROLLER_SRC_DIR})

###############################################################################
# Params orginating from qpOASES (now build as a library within this package)
###############################################################################

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE}")
add_definitions(-D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__)

####################################
# Other parameters
####################################

catkin_package(
  CATKIN_DEPENDS orunav_msgs orunav_generic roscpp sensor_msgs std_msgs nav_msgs tf
  DEPENDS Boost
)

if ((SW_BUILD_SIMULATION) AND (SW_ENABLE_NAVSERVER_READER))
    message (FATAL_ERROR "Connection to Navserver cannot be used in simulation.")
endif ()


# qpOASES
#add_subdirectory ("${PROJECT_SOURCE_DIR}/${SW_QP_SOLVER}/")
include_directories ("${PROJECT_SOURCE_DIR}/${SW_QP_SOLVER}/include/")

# compile qpOASES libraries (instead of add_subdirectory) - this in order to be able to install the qpOASES.so in the isolated install.
FILE(GLOB SW_QP_SOLVER_SRC ${SW_QP_SOLVER}/src/*.cpp)
ADD_LIBRARY(${SW_QP_SOLVER} ${SW_QP_SOLVER_SRC})


include_directories(${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Search for canlib - only needed when running on the real plattform.
if (NOT SW_BUILD_SIMULATION)
  find_library (SW_CANLIB canlib)
  if (NOT SW_CANLIB)
    message (FATAL_ERROR "Canlib is not found. Use the SW_BUILD_SIMULATION flag if you want to run the simulator.")
  endif (NOT SW_CANLIB)
endif (NOT SW_BUILD_SIMULATION)

# Maple generated functions
if (    ${SW_PREVIEW_WINDOW_LEN} EQUAL "5" OR
        ${SW_PREVIEW_WINDOW_LEN} EQUAL "15" OR
        ${SW_PREVIEW_WINDOW_LEN} EQUAL "20" OR
        ${SW_PREVIEW_WINDOW_LEN} EQUAL "25" OR
        ${SW_PREVIEW_WINDOW_LEN} EQUAL "30" )
    set (SW_FUNCGEN_SRC "${SW_FUNCGEN_SRC_DIR}/form_car_T_Sx0_${SW_PREVIEW_WINDOW_LEN}.c")
else ()
    message (FATAL_ERROR "SW_PREVIEW_WINDOW_LEN is not properly specified.")
endif ()
add_library (${SW_FUNCGEN_LIB_NAME} STATIC ${SW_FUNCGEN_SRC})
set_target_properties(${SW_FUNCGEN_LIB_NAME} PROPERTIES LINKER_LANGUAGE C)
include_directories ("${PROJECT_SOURCE_DIR}/func_gen/")

# Config file
configure_file ("${SW_CONFIG_FILE}.in"  "${SW_CONFIG_FILE}")
#set_property (DIRECTORY "${PROJECT_SOURCE_DIR}" APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES "${SW_CONFIG_FILE}")


# Executable
#     "${SW_CONTROLLER_SRC_DIR}/snowWhite.cpp"
set (SW_CONTROLLER_COMMON_SRC 
    "${SW_CONTROLLER_SRC_DIR}/state.cpp"
    "${SW_CONTROLLER_SRC_DIR}/command.cpp"
    "${SW_CONTROLLER_SRC_DIR}/commandSender.cpp"
    "${SW_CONTROLLER_SRC_DIR}/controller.cpp"
    "${SW_CONTROLLER_SRC_DIR}/qpConstraints.cpp"
    "${SW_CONTROLLER_SRC_DIR}/qpProblem.cpp"
    "${SW_CONTROLLER_SRC_DIR}/trajectory.cpp"
    "${SW_CONTROLLER_SRC_DIR}/trajectoryCache.cpp"
    "${SW_CONTROLLER_SRC_DIR}/trajectoryPool.cpp"
    "${SW_CONTROLLER_SRC_DIR}/trajectoryReader.cpp"
    "${SW_CONTROLLER_SRC_DIR}/trajectoryEvaluator.cpp"
    "${SW_CONTROLLER_SRC_DIR}/threadData.cpp"
    "${SW_CONTROLLER_SRC_DIR}/sharedAccess.cpp"
    "${SW_CONTROLLER_SRC_DIR}/swLogger.cpp")

if (SW_BUILD_SIMULATION)
    set (SW_CONTROLLER_SRC
        "${SW_CONTROLLER_COMMON_SRC}"
        "${SW_CONTROLLER_SRC_DIR}/ROSSensorSubscriber.cpp")
else (SW_BUILD_SIMULATION)
    set (SW_CONTROLLER_SRC 
        "${SW_CONTROLLER_COMMON_SRC}"
        "${SW_CONTROLLER_SRC_DIR}/canlibWrapper.cpp")
    if (SW_ENABLE_NAVSERVER_READER)
        set (SW_CONTROLLER_SRC 
            "${SW_CONTROLLER_SRC}"
            "${SW_CONTROLLER_SRC_DIR}/navserverConnection.cpp")
    else (SW_ENABLE_NAVSERVER_READER)
        set (SW_CONTROLLER_SRC 
            "${SW_CONTROLLER_SRC}"
            "${SW_CONTROLLER_SRC_DIR}/canSensorReader.cpp")
    endif (SW_ENABLE_NAVSERVER_READER)
endif (SW_BUILD_SIMULATION)

add_executable (${SW_CONTROLLER} ${SW_CONTROLLER_SRC} ${SW_CONTROLLER_SRC_DIR}/snowWhite.cpp)
add_dependencies (${SW_CONTROLLER} ${catkin_EXPORTED_TARGETS})

if (SW_BUILD_SIMULATION)
  target_link_libraries (${SW_CONTROLLER} ${SW_QP_SOLVER} ${SW_FUNCGEN_LIB_NAME} m pthread ${catkin_LIBRARIES})
else (SW_BUILD_SIMULATION)
  target_link_libraries (${SW_CONTROLLER} ${SW_QP_SOLVER} ${SW_FUNCGEN_LIB_NAME} ${SW_CANLIB} canlib m pthread ${catkin_LIBRARIES})
endif (SW_BUILD_SIMULATION)

# Flags
set_target_properties(${SW_CONTROLLER} PROPERTIES COMPILE_FLAGS "-Wall -Wfloat-equal -Wshadow")
if (NOT SW_BUILD_SIMULATION)
  set_target_properties(${SW_CONTROLLER} PROPERTIES COMPILE_FLAGS "-pedantic")
endif (NOT SW_BUILD_SIMULATION)

# ROS requires the latest C++ standard to compile without warnings. Disable some of them explicitly.
set_target_properties(${SW_CONTROLLER} PROPERTIES COMPILE_FLAGS "-Wno-variadic-macros -Wno-long-long")

#############
## Install ##
#############

install(TARGETS ${SW_CONTROLLER} ${SW_QP_SOLVER}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
