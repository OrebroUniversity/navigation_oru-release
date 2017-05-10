#set a new target for a robot
int32 task_id
int32 robot_id
int32 goal_id # JUST KEEPT FOR NOW
int32 type_id
orunav_msgs/PoseSteering start
orunav_msgs/PoseSteering goal

time start_earliest
time start_deadline
time goal_earliest
time goal_deadline

#########################################################################
# Here we use two types of loads
#########################################################################
# 1) Load that are currently on the vehicle 
# - this will be used to compute how much space the vehicle will occupy.
Load current_load

# 2) Load that we will have at the goal (after performing the goal_op)
# - this will be used to determine how to pickup the load - to compute 
# the pickup pose and the footprint used in the path planner.
Load goal_load
#########################################################################

# OLD STUFF, JUST KEEPT FOR FOR NOW
int32 carries_load

# Operation done at the start.
Operation start_op
# Operation done at the goal.
Operation goal_op
