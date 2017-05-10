# Simply hold a set of CoordinatorTimes to avoid transfering one at a time

# ID of the robot
int32 robot_id # depreciated

# ID of the goal
int32 goal_id # depreciated

# Unique incremental ID used in case the msg needs to be updated (due to replanning / rescheduling)
int32 id # depreciated

#####################

CoordinatorTime[] ts
