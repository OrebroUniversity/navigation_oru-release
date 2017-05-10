# ID of the robot
int32 robot_id

#####################

# ID of the trajectory
int32 traj_id

#####################

# Sequential number of the chunk, should start from 0 and be incremented by 1 for each chunk
# It is assumed that overflows do not occur.
uint32 sequence_num

#####################

# True if the chunk is final (the end of a trajectory), false otherwise.
bool final

#####################

# Constraints, which are imposed on the steps in this chunk of trajectory
ControllerConstraints constraints

#####################

# Steps (non-empty)
ControllerTrajectoryStep[] steps
