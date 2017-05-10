# ID of the robot
int32 robot_id
#####################

time stamp

# The car is waiting, since there is no data to start tracking or the end of trajectory is reached
int32 CONTROLLER_STATUS_WAIT = 1
# Tracking was interrupted due to an error or by request
int32 CONTROLLER_STATUS_FAIL = 2
# A trajectory is being tracked
int32 CONTROLLER_STATUS_ACTIVE = 3
# The end of trajectory is reached, controller performs final corrections
int32 CONTROLLER_STATUS_FINALIZE = 4
# Controller is terminated
int32 CONTROLLER_STATUS_TERMINATE = 5

# Status of the controller.
int32 status

#####################

# The current state of a car.
ControllerState state

#####################

# Results of trajectory assessment, may be empty.
ControllerTrajectoryValue[] traj_values

#####################

# The current trajectory sequence numbers, only valid if the controller status is active
uint32 traj_chunk_sequence_num
uint32 traj_step_sequence_num