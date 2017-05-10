# This data is supposed to be generated from a perception system the overlooks the load

# ID of the robot
int32 robot_id

# Estimated state of the forks - the height
ForkState state

# What type of load 
Load load

#############################
# Relative pose of the load - this is to be able to unload the cargo more accuractly. 
# The idea is to utilize this information while computing the final delivery pose.
geometry_msgs/Pose pose
