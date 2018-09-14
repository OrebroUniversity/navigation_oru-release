^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cititruck_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2018-09-11)
------------------
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Merge pull request `#27 <https://github.com/OrebroUniversity/navigation_oru-release/issues/27>`_ from batman177/master
  Cititruck calibration
* Merge branch 'master' into master
* Merge pull request `#28 <https://github.com/OrebroUniversity/navigation_oru-release/issues/28>`_ from MFernandezCarmona/master
  Working version in NCFM cititruck4
* error in velodyne tf
* merge Orebro with my branch
* Added dependence to asus_description. Changed frame name to match iliad_velodine_xacro
* Merge branch 'master' of https://github.com/MFernandezCarmona/navigation_oru-release
* Added support for gazebo8-gpu-accelerated simulated velodyne on L-CAS repos. Name refactoring
* Saving progress
* Merge upstream branch 'master'
* Adding sensor calibration joints for Cititruck. Some cleanup.
* Removing calibration files, now in iliad_launch_system
* Merge branch 'master' from origin
* Removing bt_truck.launch, moved to iliad_metapackage/iliad_launch_system
* Merge pull request `#1 <https://github.com/OrebroUniversity/navigation_oru-release/issues/1>`_ from OrebroUniversity/master
  Merge from Orebro
* added simulated asus
* Contributors: Brucye Wayne, Henrik Andreasson, Manuel Fernandez-Carmona

0.3.0 (2018-08-24)
------------------
* post IJCAI demo, added some maps and launch files for arena
* IJCAI demo map
* Fixed the last launch example with the pallet picking.
* Getting single_truck.launch from the tutorial to run again.
* Merge pull request `#25 <https://github.com/OrebroUniversity/navigation_oru-release/issues/25>`_ from batman177/master
  Rough extrinsic calibration (sensors against each other) of Velodyne, Asus, Kinect v2, 2D lasers
* Extrinsic calibration of Asus on bt_truck. Intrinsic calibration still needs to be improved, z scaling is off
* Rough Kinect v2 calibration on bt_truck, had to add one extra joint, needs to be verified on real hardware
* In calibration macro/script, make final sensor link configurable (for Kinect v2)
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Merge pull request `#24 <https://github.com/OrebroUniversity/navigation_oru-release/issues/24>`_ from MFernandezCarmona/master
  Parametric names in topics and in xacro files
* Merge pull request `#22 <https://github.com/OrebroUniversity/navigation_oru-release/issues/22>`_ from batman177/master
  Update bt_truck URDF, sensor naming conventions, rough calibration WIP
* Pushing results from integration week: Fixing calibration when using fixed joints; adding save_calibration script; renaming LS2000->laser_top; adding calibration joint for laser2d_top; first very rough calibration of Velodyne and 2D lasers on bt_truck.
* remapping fork control topics
* parametrized topics in xacro files
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Merge branch 'master' into master
* Extend rotation range of RPY sensor calib joints
* Apply new naming scheme to sensors of bt_truck
* Renaming btruck to bt_truck to adhere to naming conventions
* Merge upstream master branch into master
* Saving integration week changes, updating launch files for BTruck
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* xacro model with forklift cad
* Modified RVIZ config file for 2 cititrucks
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Merge pull request `#20 <https://github.com/OrebroUniversity/navigation_oru-release/issues/20>`_ from batman177/master
  Improvements to BT URDF + Gazebo launch files
* BT joint state publisher GUI off per default
* Rename btruck launch files to lower case for consistency; split up into URDF launcher + Gazebo launcher, to be able to run former on the robot.
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Merge pull request `#19 <https://github.com/OrebroUniversity/navigation_oru-release/issues/19>`_ from batman177/master
  Adding sensor calibration for big truck
* Add missing calibration folder
* Adding sensor calibration for big truck
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Urdf model of the BTruck forklift with sensors
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* BT truck urdf model - work in progress
* Changed RVIZ config files to include markers from coordination_oru_ros
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* modified cititruck description for kinect camera in simulation, testing pallet detection
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Changed the cmd_vel default topic used.
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Contributors: Bruce Wayne, Brucye Wayne, Federico Pecora, Henrik Andreasson, Joao Salvado, Manuel Fernandez-Carmona, tsv

0.2.2 (2018-01-18)
------------------
* Made the navigation laser and safety laser to have the same FOV to get the raytracing to work correctly (using ray_gpu + a NVIDIA card).
* Added rviz cfg for ncfm example with three trucks
* Add Kinect v2 simulation. In iliad_single_truck.launch, add options for enabling/disabling simulation of Kinect v1, v2 and Velodyne sensors.
* Updated DEPENDS and CATKIN_DEPENDS.
* showing the costmap_2D visualization, adding the possibility to read the velodyne point clouds into the smp motion planner
* Add option to enable human perception in iliad_single_truck.launch. Fix Velodyne ns in Rviz.
* Updated Rviz config to include groundtruth tracked persons
* Refactored launch and config files to better handle multiple truck launches
* added a map for simulation
* Fix size and position of empty map for NCFM; disable Velodyne visualization
* Contributors: Bruce Wayne, Federico Pecora, Henrik Andreasson, Palmieri Luigi (CR/AEG) VM, tsv

0.2.1 (2017-09-19)
------------------
* Merge pull request `#3 <https://github.com/OrebroUniversity/navigation_oru-release/issues/3>`_ from batman177/master
  Add LS2000 laser scanner to ILIAD cititruck etc.
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Add LS2000 laser scanner; update Rviz config; fix robot initial position
* merge
* Merge branch 'master' of github.com:OrebroUniversity/navigation_oru-release
* decreased frequency for odometry
* Fix XML formatting and whitespaces
* fixed some of namespace problems
* added launchfile for illiad single truck
* Contributors: Bruce Wayne, Daniel Adolfsson, Martin Magnusson, dan11003, tsv

0.2.0 (2017-09-15)
------------------

0.1.1 (2017-06-13)
------------------

0.1.0 (2017-06-13)
------------------

0.0.10 (2017-06-12)
-------------------

0.0.9 (2017-06-09)
------------------

0.0.8 (2017-06-08)
------------------

0.0.7 (2017-06-08)
------------------

0.0.6 (2017-06-08)
------------------

0.0.5 (2017-06-07)
------------------

0.0.4 (2017-06-07)
------------------

0.0.3 (2017-06-07)
------------------

0.0.2 (2017-05-25)
------------------

0.0.1 (2017-05-23)
------------------
* Added missing install files.
* Adding install files.
* Initial version.
* Contributors: Henrik Andreasson
