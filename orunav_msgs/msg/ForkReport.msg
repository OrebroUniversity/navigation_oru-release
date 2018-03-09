# ID of the robot
int32 robot_id
#####################

time stamp

# Forks are in known position given by the height value (default, what is below is focused if forks heights are not available -> the citi trucks)
int32 FORK_POSITION_KNOWN = 0
# Forks are in unknown position
int32 FORK_POSITION_UNKNOWN = 1
# Forks are in 'low' position - we can move in below a pallet from here
int32 FORK_POSITION_LOW = 2
# Forks are in a 'high' postion - we can drive holding a pallet.
int32 FORK_POSITION_HIGH = 3
# Forks can be lowered to be used as support legs (for the arm)
int32 FORK_POSITION_SUPPORT_LEGS = 4

# Moving up
int32 FORK_MOVING_UP = 5
# Moving down
int32 FORK_MOVING_DOWN = 6
# Failure
int32 FORK_FAILURE = 7

# Status of the fork controller.
int32 status

#####################

# The current state
ForkState state

#####################
