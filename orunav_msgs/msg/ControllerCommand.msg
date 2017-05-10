# ID of the robot
int32 robot_id

#####################

# Stop the car
int32 COMMAND_BRAKE = 1
# Set or change the active trajectory
int32 COMMAND_ACTIVATE = 2
# Set start time of tracking
int32 COMMAND_STARTTIME = 3
# Recover after failure, ignored if there was no failure.
int32 COMMAND_RECOVER = 4

# Status of the controller.
int32 command

#####################

# Must be initialized for COMMAND_STARTTIME
time start_time

#####################

# ID of the trajectory, must be initialized for COMMAND_ACTIVATE
int32 traj_id
