# All robots have a /robotX/ebrake topic availble. Everybody can stop a robot through this topic. In order to allow the vehicle to start driving another ebrake command has to be tunned using the recover flag = true.

# ID of the robot
int32 robot_id

# Unique sender ids
int32 SENDER_ID_vSMU = 1
int32 SENDER_ID_X = 2
int32 SENDER_ID_Y = 3


# ID of the sender, to avoid that multiple "processes" utilize the same sender_id, add your id number above.
int32 sender_id

# Set this active in order to recover
bool recover
