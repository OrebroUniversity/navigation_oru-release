# ID of the robot
int32 robot_id
#####################

time stamp

# Jacks are in unknown position
int32 JACK_POSITION_UNKNOWN = 1
# Jacks are in 'low' position - we can move in below a pallet from here
int32 JACK_POSITION_EXTENDED = 2
# Jacks are in a 'high' postion - we can drive holding a pallet.
int32 JACK_POSITION_RETRACKTED = 3

# Moving up
int32 JACK_MOVING_IN = 4
# Moving down
int32 JACK_MOVING_OUT = 5
# Failure
int32 JACK_FAILURE = 6

# Status of the Jack controller.
int32 status

#####################

# The current state
JackState state

#####################
