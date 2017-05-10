## Installation instructions

This is a brief guide to installing the ROS-based software tools NAVIGATION_ORU developed at ORU in a set of previous project (SAUNA, SAVIE, Semantic Robot). The tools require full ROS installation and two external packages. The installation assumes you have Ubuntu 16.04 LTS [ROS Kinetic] or Ubntu 14.04 LTS [ROS Indigo].

###### Install ROS:

Please refer to http://wiki.ros.org/kinetic/Installation/Ubuntu

###### Install map server

$ sudo apt-get install ros-kinetic-map-server 

###### Get the navigation_oru source tree:

Download the navigation_oru source tree into your catkin workspace (here we assume ~/catkin_ws):

$ cd ~/catkin_ws/src
$ git clone https://github.com/OrebroUniversity/navigation_oru-release.git navigation_oru

Do not compile yet... please follow the instructions below first...

###### Install external dependencies:

Two external external packages are required: ACADO Toolkit (provided in the source tree) and Meta-CSP Framework (available on GitHub).

- ACADO Toolkit (used for the orunav_path_smoother)

You need to build this package from source.

$ git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
$ cd ACADOtoolkit
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

(feel free to remove the ACADOtoolkit tree now)


###### Compile the orunav source tree

$ cd ~/catkin_ws
$ catkin_make						#Read below before catkin_make

NOTE: the default build type is in DEBUG mode - some packages as the constraint extractor work way faster if they are built in RELEASE mode. Change the build into RELASE mode by:
$ catkin_make -DCMAKE_BUILD_TYPE=Release


###### Gazebo simulation environment

Update the GAZEBO_MODEL_PATH variable, add the following to your ~/.bashrc file:

export GAZEBO_MODEL_PATH=/home/<your user name>/catkin_ws/src/navigation_oru/gazebo_oru/models:$GAZEBO_MODEL_PATH

This is needed inorder to be able to load the different environements (otherwise you get "no namepace found" messages and an empty world).

