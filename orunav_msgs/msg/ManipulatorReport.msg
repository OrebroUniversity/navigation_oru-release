# ID of the robot
int32 robot_id
#####################

time stamp

# Status of the fork controller.
int32 status
int32 item_id

#####################

# Manipulator is not mounted on the vehicle, or not operational
int32 NOT_AVAILABLE = 1

int32 IDLE = 2

int32 LOADING_ITEM = 3

int32 UNLOADING_ITEM = 4

int32 UNWRAP_PHASE_1 = 5

int32 UNWRAP_PHASE_2 = 6

int32 HOMING = 7

# Manipulator has undergone some failure
int32 FAILURE = 8


