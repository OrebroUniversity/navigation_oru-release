# ID of the robot
int32 robot_id
#####################

time stamp

# Status of the fork controller.
int32 status

#####################

# The current state
ManipulatorState state

#####################

# Manipulator is not mounted on the vehicle, or not operational
int32 MANIPULATOR_NOT_AVAILABLE = 1

int32 MANIPULATOR_LOAD_ITEM_START = 1

int32 MANIPULATOR_LOAD_ITEM_DONE = 2

int32 MANIPULATOR_UNLOAD_ITEM_START = 3

int32 MANIPULATOR_UNLOAD_ITEM_DONE = 4 

int32 MANIPULATOR_UNWRAP_PALLET_START = 5

int32 MANIPULATOR_UNWRAP_PALLET_DONE = 6

int32 MANIPULATOR_GOTO_IDLE_START = 7

int32 MANIPULATOR_GOTO_IDLE_DONE = 8

int32 MANIPULATOR_GOTO_HOME_START = 9

int32 MANIPULATOR_GOTO_HOME_DONE = 10

# Manipulator has undergone some failure
int32 MANIPULATOR_FAILURE = 8
