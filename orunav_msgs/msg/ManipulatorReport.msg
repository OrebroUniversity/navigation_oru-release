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
int32 MANIPULATOR_NO_OPERATION = 2
# Manipulator going to homing position
int32 MANIPULATOR_HOMING = 3

# Manipulator performing loading of an item
int32 MANIPULATOR_LOAD_OBJECT = 4
# Manipulator performing unloading of an item
int32 MANIPULATOR_UNLOAD_OBJECT = 5

# Manipulator performing unwrapping of pallet
int32 MANIPULATOR_UNWRAPPING = 6

# Manipulator has undergone some failure
int32 MANIPULATOR_FAILURE = 7