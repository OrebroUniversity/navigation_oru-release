int32 COMPUTE_TASK_START = 1

# Necessary conditions to compute a task
int32 INVALID_TASK_ID = 2
int32 INVALID_MAP = 3
int32 INVALID_START = 4

# Vector map service called
int32 VECTOR_MAP_CALL_SUCCESS = 5

# Failure calling vector map service
int32 VECTOR_MAP_CALL_FAILURE = 6

# Geofence map service called
int32 GEOFENCE_CALL_SUCCESS = 7

# Failure calling vector map service
int32 GEOFENCE_CALL_FAILURE = 8

# Motion planning service called
int32 PLANNING_SUCCESS = 9

# Failure calling motion planning service
int32 PLANNING_CALL_FAILURE = 10

# No path found but target and and start are far enough
int32 PLANNING_FAILURE = 11

# Failing to compute repositioning
int32 PLANNING_FAILURE_REPOSITIONING = 12

# Path smoother service called
int32 SMOOTHING_SUCCESS = 13

# Failure calling smoothing service
int32 SMOOTHING_CALL_FAILURE = 14

# Failure of the constraint extractor
int32 SMOOTHING_CONSTRAINT_FAILURE = 15

# DeltaTVec service successfully called
int32 DELTATVEC_SUCCESS = 16

# Failure calling DeltaTVec service 
int32 DELTATVEC_CALL_FAILURE = 17

# Failure of the DeltaTVec constraint extractor
int32 DELTATVEC_CONSTRAINT_FAILURE = 18

# Received compute task
int32 COMPUTE_TASK_ENDED = 19

int32 task_id
int32 status

