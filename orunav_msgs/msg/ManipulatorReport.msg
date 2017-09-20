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

# Manipulator is idle
int32 MANIPULATOR_IDLE = 2

# Manipulator going to homing position
int32 MANIPULATOR_HOMING = 3

# Manipulator performing loading of an item
int32 MANIPULATOR_LOADING_ITEM = 4
# Manipulator is carrying an object, full hands
int32 MANIPULATOR_FULL = 5
# Manipulator performing unloading of an item
int32 MANIPULATOR_UNLOADING_ITEM = 6

# Manipulator performing unwrapping of pallet
int32 MANIPULATOR_UNWRAPPING = 7

# Manipulator has undergone some failure
int32 MANIPULATOR_FAILURE = 8
