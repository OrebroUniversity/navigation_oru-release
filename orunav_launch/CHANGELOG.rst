^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package orunav_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2018-01-18)
------------------
* Safetyregions flag added, with additional params.
* Added brake release option to motion_planning_and_control_smp.launch
* Added brake release option to motion_planning_and_control.launch
* Add Kinect v2 simulation. In iliad_single_truck.launch, add options for enabling/disabling simulation of Kinect v1, v2 and Velodyne sensors.
* adding param to avoid smoothin step when using smp
* Updated DEPENDS and CATKIN_DEPENDS.
* having velodyne param set to false as default value
* showing the costmap_2D visualization, adding the possibility to read the velodyne point clouds into the smp motion planner
* Refactored launch files
* Delete human agents from NCFM scene if human_perception is set to false (as per default)
* Add option to enable human perception in iliad_single_truck.launch. Fix Velodyne ns in Rviz.
* Make coordination_oru_ros optional (launch via iliad_single_truck.launch coordination:=false)
* Refactored launch and config files to better handle multiple truck launches
* added a map for simulation
* Contributors: Bruce Wayne, Federico Pecora, Henrik Andreasson, Palmieri Luigi (CR/AEG) VM, Palmieri Luigi (CR/AER), tsv

0.2.1 (2017-09-19)
------------------
* Merge pull request `#3 <https://github.com/OrebroUniversity/navigation_oru-release/issues/3>`_ from batman177/master
  Add LS2000 laser scanner to ILIAD cititruck etc.
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* Add LS2000 laser scanner; update Rviz config; fix robot initial position
* moved truck to a more drivable place, added an empty map to allow more movement
* merge
* Merge branch 'master' of github.com:OrebroUniversity/navigation_oru-release
* fixed some of namespace problems
* Merge branch 'master' of https://github.com/OrebroUniversity/navigation_oru-release
* added launchfile for illiad single truck
* Added launch file in orunav_launch that shows the use of iliad_smp motion planner
* Contributors: Bruce Wayne, Daniel Adolfsson, Federico Pecora, Martin Magnusson, dan11003, tsv

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
* Cleanup, fixing names removing un-used files.
* Adding install files.
* Added install section.
* Updated the installation instuctions, added a few examples to run.
* Initial version.
* Contributors: Henrik Andreasson
