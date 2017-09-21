# ID of the robot
int32 robot_id

# Command
int32 cmd

int32 MANIPULATOR_LOAD = 1

int32 MANIPULATOR_UNLOAD = 2

int32 MANIPULATOR_UNWRAP = 3

int32 MANIPULATOR_GOTO_IDLE = 4

int32 MANIPULATOR_GOTO_HOME = 5

# Commanded state
ManipulatorState state

# pose for unloading
geometry_msgs/Pose item_pose