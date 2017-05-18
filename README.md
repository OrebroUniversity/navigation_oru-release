# navigation_oru (orunav) navigation stack

This contains a breif guide how to install / run some examples of the ROS-based software tools NAVIGATION_ORU developed at ORU in a set of previous project (SAUNA, SAVIE, Semantic Robot).

## Installation instructions

The tools require full ROS installation and one external packages (ACADO). The installation assumes you have Ubuntu 16.04 LTS [ROS Kinetic] or Ubntu 14.04 LTS [ROS Indigo].

#### Install ROS:

Please refer to http://wiki.ros.org/kinetic/Installation/Ubuntu

#### Install map server

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

(feel free to remove the ACADOtoolkit tree now)


#### Compile the orunav source tree

`$ cd ~/catkin_ws`

`$ catkin_make`						


NOTE: the default build type is in DEBUG mode - some packages as the constraint extractor work way faster if they are built in RELEASE mode. Change the build into RELASE mode by:

`$ catkin_make -DCMAKE_BUILD_TYPE=Release`


#### Gazebo simulation environment

Update the GAZEBO_MODEL_PATH variable, add the following to your ~/.bashrc file:

`export GAZEBO_MODEL_PATH=/home/<your user name>/catkin_ws/src/navigation_oru/gazebo_oru/models:$GAZEBO_MODEL_PATH`

This is needed inorder to be able to load the different environements (otherwise you get "no namepace found" messages and an empty world).


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


Similar to the single truck example you can utilize the task bar to select where each truch should drive. Note that the coordination is not in place yet and you can easily make the trucks to collide. To set which truck should go where use one of the "2D Nav Goal" buttons (there are three of them, the first will trigger the first vehicle, the second the second vehicle and so on).
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

The alignement of the pallets is done using a sdf based tracker - due to the camera monting the images are up-side down in the visualization.

![alt text][sdf_pallet_model]
![alt text][sdf_pallet_aligned]


[sdf_pallet_model]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/sdf_pallet_model.png "SDF model of an EUR-pallet"
[sdf_pallet_aligned]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/sdf_pallet_aligned.png "The aligned pallet"
[rviz_pallet_picking]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/rviz_pallet_picking.png "RViz visualization of pallet picking, the red box is the intial pallet pose estimate and the green one is the estimated one."
[gazebo_pallet_picking]: https://github.com/OrebroUniversity/navigation_oru-release/blob/master/docs/figs/gazebo_pallet_picking.png "Gazebo view of the pallet picking."
