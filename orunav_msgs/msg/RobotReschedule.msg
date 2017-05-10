#set a new reschedule request for a vehicle, this can only be used with an existing target (goal_id).
int32 robot_id
int32 goal_id
int32 id

# the start pose (should lie on the path)
orunav_msgs/PoseSteering start

time start_earliest
time start_deadline
