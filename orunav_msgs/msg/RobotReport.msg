# ID of the robot
int32 robot_id
#####################

time stamp

#WAITING_FOR_TASK = 1, PERFORMING_START_OPERATION, DRIVING, PERFORMING_GOAL_OPERATION, TASK_FAILED, WAITING_FOR_TASK_INTERNAL, DRIVING_SLOWDOWN, AT_CRITICAL_POINT
int32 WAITING_FOR_TASK = 1
int32 PERFORMING_START_OPERATION = 2
int32 DRIVING = 3
int32 PERFORMING_GOAL_OPERATION = 4
int32 TASK_FAILED = 5
int32 WAITING_FOR_TASK_INTERNAL = 6
int32 DRIVING_SLOWDOWN = 7
int32 AT_CRITICAL_POINT = 8

# The vehicle is waiting for a task - the controller is in wait state
#int32 WAITING_FOR_TASK = 1
# Performing load operations - controller is in wait state
#int32 PERFORMING_START_OPERATION = 2
# Driving - the controller can be in all states but not in wait
#int32 DRIVING = 3
# The end of trajectory is reached, performing goal operations - the controller is in wait state.
#int32 PERFORMING_GOAL_OPERATION = 4
# Task failed - the requested task is not valid / possible to perform.
#int32 TASK_FAILED = 5

# Status of the robot.
int32 status

# Number of active brake reasons (=0 no brake is active)
int32 nb_active_brake_reasons

#####################

# The current state
PoseSteering state

#####################

# The current path / constraint / coordination time sequence number, only valid if the vehicle is either driving or performing start / goal operations
uint32 sequence_num
