^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cititruck_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
