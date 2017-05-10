# ID of the trajectory
int32 traj_id

#####################

bool active

#####################

# Ok.
int32 VALUE_OK = 1
# QP was failed
int32 VALUE_FAIL = 2
# Not enough data to form problem.
int32 VALUE_NODATA = 3

# Status
int32 status

#####################

# Value of the objective function
# (weighted sum of deviations from the reference states and controls)
float64 value
