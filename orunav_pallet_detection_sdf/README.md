# Pallet Detection and Localization in 3D Space. <br />

## [Demo video](https://www.youtube.com/watch?v=Cd79mRIMDks)

### The code is tested on Ubuntu 16.04, ROS kinetic, tensorflow gpu version 1.6, Cuda 9.0, Asus Xtion Pro Live Camera. <br />

$ cd ~/catkin_ws <br />
$ source devel/setup.bash <br />
$ cd ~/catkin_make <br />
$ catkin_make --pkg  orunav_pallet_detection_sdf <br />
$ roslaunch orunav_pallet_detection_sdf pallet_detector_node.launch <br />

Segmantation:

![alt text](https://github.com/OrebroUniversity/navigation_oru-release/blob/ms2-wip/orunav_pallet_detection_sdf/docs/images/segmentation.png)

Model Registration:

![alt text]( https://github.com/OrebroUniversity/navigation_oru-release/blob/ms2-wip/orunav_pallet_detection_sdf/docs/images/model_matching.png )

Pose Estimation:

![alt text](https://github.com/OrebroUniversity/navigation_oru-release/blob/ms2-wip/orunav_pallet_detection_sdf/docs/images/pose_estimation.png)
