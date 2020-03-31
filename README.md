# navigation_oru (orunav) navigation stack

This contains a brief guide how to install / run some examples of the ROS-based software tools NAVIGATION_ORU developed at ORU in a set of previous and on-going projects (SAUNA, SAVIE, ILIAD and Semantic Robot).

## Installation instructions

The tools require full ROS installation and one external packages (ACADO). The installation assumes you have Ubuntu 18.04 LTS [ROS Melodic] or Ubuntu 16.04 LTS [ROS Kinetic] (or Ubntu 14.04 LTS [ROS Indigo]).

#### Install ROS:

Please refer to  http://wiki.ros.org/melodic/Installation/Ubuntu or http://wiki.ros.org/kinetic/Installation/Ubuntu

#### Install map server

`$ sudo apt-get install ros-melodic-map-server`
or
`$ sudo apt-get install ros-kinetic-map-server`


#### Get the navigation_oru source tree:

Download the navigation_oru source tree into your catkin workspace (here we assume ~/catkin_ws):

`$ cd ~/catkin_ws/src`

`$ git clone https://github.com/OrebroUniversity/navigation_oru-release.git navigation_oru`

Do not compile yet... please follow the instructions below first...

#### Install external dependencies:

- ACADO Toolkit (used for the orunav_path_smoother)

You need to build this package from source.

`$ git clone https://github.com/acado/acado.git -b stable ACADOtoolkit`

`$ cd ACADOtoolkit`

`$ mkdir build`

`$ cd build`

`$ cmake ..`

`$ make`

`$ sudo make install`

`$ sudo ldconfig`

(feel free to remove the ACADOtoolkit tree now)


#### Compile the orunav source tree

`$ cd ~/catkin_ws`

`$ catkin_make`						


NOTE: the default build type is in DEBUG mode - some packages as the constraint extractor work way faster if they are built in RELEASE mode. Change the build into RELEASE mode by:

`$ catkin_make -DCMAKE_BUILD_TYPE=Release`


## A set of examples

#### A single truck - basement environment of AASS

`$ roslaunch orunav_launch single_truck.launch`

This should bring up the gazebo simulation GUI and RViz.
![alt text][gazebo_single_truck]
![alt text][rviz_single_truck]

In order to navigate to a pose in the environment, select the "2D Nav Goal" button on the task bar in RViz![alt text][rviz_task_bar] and point/click in the map where the vehicle should drive. First orunav_motion_planner (a lattice-based motion planner) generates an initial path (green) which then are optimized using orunav_path_smoother which then are executed automatically by the orunav_mpc tracking controller. This is how it typically should look like.
![alt text][rviz_single_truck_run]

[gazebo_single_truck]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/gazebo_single_truck.png "Gazebo simulation of the AASS basement with a single forklift"
[rviz_single_truck]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/rviz_single_truck.png "RViz visualization of the AASS basement with a single forklift"
[rviz_task_bar]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/rviz_task_bar.png "RViz task bar"
[rviz_single_truck_run]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/rviz_single_truck_run.png "RViz visualization of a sample run"


#### A set of three trucks - empty environment

`$ roslaunch orunav_launch multiple_trucks.launch`    

This will bring up the gazebo simulation GUI and RViz. There we have an empty environment with three trucks. 


Similar to the single truck example you can utilize the task bar to select where each truck should drive. Note that the coordination is not running here, check out https://github.com/FedericoPecora/coordination_oru if you want coordination as well. To set which truck should go where use one of the "2D Nav Goal" buttons (there are three of them, the first will trigger the first vehicle, the second the second vehicle and so on).
![alt text][rviz_multiple_trucks_run].

[rviz_multiple_trucks_run]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/rviz_multiple_trucks_run.png "RViz visualization of a sample run with three vehicles"


#### Pallet picking from an semi-known pose

This is based on some old stuff that haven't been used for a while and is not really robust in the current state. The idea here is to pickup a pallet at a semi-known pose. To bring up the simulation / visualization run:

`$ roslaunch orunav_launch click_n_pick.launch`

Two goal target poses is loaded from a file, one which is the approximate pickup location and the other is the drop of location. To send the pickup command run:

`$ rosservice call /robot1/next_task`

![alt text][rviz_pallet_picking] 
![alt text][gazebo_pallet_picking] 

...and to leave the pallet call the same service again:

`$ rosservice call /robot1/next_task`

The alignment of the pallets is done using a sdf based tracker - due to the camera mounting the images are up-side down in the visualization.

![alt text][sdf_pallet_model]
![alt text][sdf_pallet_aligned]


[sdf_pallet_model]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/sdf_pallet_model.png "SDF model of an EUR-pallet"
[sdf_pallet_aligned]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/sdf_pallet_aligned.png "The aligned pallet"
[rviz_pallet_picking]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/rviz_pallet_picking.png "RViz visualization of pallet picking, the red box is the initial pallet pose estimate and the green one is the estimated one."
[gazebo_pallet_picking]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/gazebo_pallet_picking.png "Gazebo view of the pallet picking."


## Description of packages

In alphabetic order.

#### gazebo_oru

Meta package containing plugins, vehicle models, worlds etc.

#### navigation_oru

Meta package for the whole stack.

####  orunav_constraint_extract

Given an occupancy map, the geometry of the vehicle, and a vehicle pose, this package computes a collision free region (area and heading interval). This is used in the orunav_path_smoother package to optimize the path.

#### orunav_conversions

Package to handle the conversion between generic messages and types. Note that some conversions of more specific types is defined in the package defining the type.

#### orunav_coordinator_fake

This provides an coordinator instance which have the same interface as the real coordinator but doesn't do any coordination. This return the fastest trajectory for all vehicles.

#### orunav_debug

Package with debugging tools. There is a tool to plot the behavior of the controller, see test/README.txt in the package.

#### orunav_fork_control

Provides an interface to operating the forklifts. This interface is used for driving the trucks in gazebo and the real CitiTrucks.

#### orunav_generic

A set of generic types and interfaces, such as pose, path, trajectory etc. Contains also various of utilities function and also ways to do serialization (saving / loading).

#### orunav_geometry

2D geometry, mainly polygon intersection etc. for computing vehicles footprints.

#### orunav_launch

Contains a set of example launch files.

#### orunav_motion_planner

Lattice based motion planner, will provide a relative fast way of computing kinetically feasible motion between discretized poses.

#### orunav_mpc

Tracking controller using model predictive control. Handles apart from the trajectory also constraints on poses along the path.

#### orunav_msgs

Package for all specialized messaged used.

#### orunav_node_utils

Generic tools that are useful when writing nodes to handle a set of targets / missions.

#### orunav_pallet_detection_sdf

Package using signed distance function to estimate poses of euro pallets.

#### orunav_params

Package that contains a set of parameters for the system.

#### orunav_path_pool

Path planning using a set of predefined paths.

#### orunav_path_smoother

Path smoothing functionality that takes an existing motion, for example, from the orunav_motion_planner, and minimize the amount of required driving and turning.

#### orunav_rosbag_tools

Rosbag processing tools, contains an odometry meter.

#### orunav_rviz

Functions to display various of generic types (orunav_generic) to rviz. Note that some rviz message generation for specific types are located in the package where they are defined.

#### orunav_trajectory_processor

Given a path and acceleration, max velocity, etc.  constraints compute a speed profile for the path.

#### orunav_vehicle_execution

Package that acts as an interface to the tracking controller (orunav_mpc) and the coordinator. Contains the interface to compute task (e.g. how drive to this location) and execute task.

## Citations

The publication that to the highest degree explains the navigation_oru stack (most suitable citing) is:

> H. Andreasson, J. Saarinen, M. Cirillo, T. Stoyanov and A. Lilienthal, “Fast, continuous state path smoothing to improve navigation accuracy”, ICRA 2015 [link](https://www.researchgate.net/profile/Henrik_Andreasson/publication/283778029_Fast_continuous_state_path_smoothing_to_improve_navigation_accuracy/links/582ab92f08ae102f071fd502/Fast-continuous-state-path-smoothing-to-improve-navigation-accuracy.pdf).
