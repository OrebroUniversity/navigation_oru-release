# Pallet Detection and Localization in 3D Space.
<br />
## A demo video can be found here [Demo Video](https://www.youtube.com/watch?v=Cd79mRIMDks).
<br />
### The code is tested on Ubuntu 16.04, ROS kinetic, tensorflow gpu version 1.6, Cuda 9.0, Asus Xtion Pro Live Camera. <br />



<br />
$ cd ~/catkin_ws <br />
$ source devel/setup.bash <br />
$ cd ~/catkin_make <br />
$ catkin_make --pkg  orunav_pallet_detection_sdf <br />
$ roslaunch orunav_pallet_detection_sdf pallet_detector_node.launch <br />

<br />
![Recognition](https://gitsvn-nt.oru.se/cghg/navigation_oru/blob/master/orunav_pallet_detection_sdf/docs/images/segmentation.png)
<br />
![Model Matching](https://gitsvn-nt.oru.se/cghg/navigation_oru/blob/master/orunav_pallet_detection_sdf/docs/images/model_matching.png)
<br />
![Pose Estimation](https://gitsvn-nt.oru.se/cghg/navigation_oru/blob/master/orunav_pallet_detection_sdf/docs/images/pose_estimation.png)

