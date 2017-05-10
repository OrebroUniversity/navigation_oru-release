# Duration of one step is 60 millisecond.
#
# The first step of a trajectory must include the current state as the reference state and
# zero velocities. 
#
# Note that steering angle should stay within (-pi/2,pi/2) due to singularity in the model 
# of a car. However, controller does not fail if this does not hold, it simply executes a 
# smaller angle.


# Reference state / control
ControllerState state
# These velocities must be applied to the preceding state to reach the respective 
# reference state. 
ControllerVelocities velocities

#####################

# Tracking mode
# Tracking modes allow to assign different sets of gains to points in order to penalize 
# devitation of certain state or control variables from the reference ones. Currently two 
# modes are supported. The gains for each mode must be specified in the configuration file.
int32 MODE_1 = 1
int32 MODE_2 = 2
int32 mode
