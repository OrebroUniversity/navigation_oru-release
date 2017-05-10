# This can be used to brake the vehicle. This will trigger a brake command to be sent to the controller (using the vehicle_executor node) - using topic /robotX/brake_cmd
# The vehicle will stay in BRAKE state until it recives a new msg where the brake flag is set to RELEASE_BRAKE.
####################################
# ID of the robot
int32 robot_id

time stamp

# Brake command issued by a safety scanner
int32 BRAKE_ID_SICK = 1

# Brake command issued by the Velodyne tracking method (Abdel & Martin)
int32 BRAKE_ID_VELODYNE_TRACKER = 2

# ID of the sender of the command
int32 sender_id


int32 BRAKE = 1
int32 RELEASE_BRAKE = 2
int32 SLOW_DOWN = 3
int32 brake_flag
