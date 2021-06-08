# Received compute task
int32 COMPUTE_TASK_CALL_SUCCESS = 1

# Vector map service called
int32 VECTOR_MAP_CALL_SUCCESS = 2

# Failure calling vector map service
int32 VECTOR_MAP_CALL_FAILURE = 3

# Geofence map service called
int32 GEOFENCE_CALL_SUCCESS = 4

# Failure calling vector map service
int32 GEOFENCE_CALL_FAILURE = 5

# Motion planning service called
int32 PLANNING_SUCCESS = 6

# Failure calling motion planning service
int32 PLANNING_CALL_FAILURE = 7

# No path found
int32 PLANNING_PATH_FAILURE = 8

# Path smoother service called
int32 SMOOTHER_SUCCESS = 9

# Failure calling smoothing service
int32 SMOOTHER_CALL_FAILURE = 10

# Failure of the constraint extractor
int32 SMOOTHER_CONSTRAINT_FAILURE = 11

# DeltaTVec service successfully called
int32 DELTATVEC_SUCCESS = 12

# Failure calling DeltaTVec service 
int32 DELTATVEC_CALL_FAILURE = 13

# Failure of the DeltaTVec constraint extractor
int32 DELTATVEC_CONSTRAINT_FAILURE = 14

# Received compute task
int32 COMPUTE_TASK_ENDED = 15

int32 task_id
int32 status

