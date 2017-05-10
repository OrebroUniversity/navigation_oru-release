# Constraints, which are imposed on certain steps of a trajectory.
# Any of these constraints can be omitted, in this case default values from the
# configuration file are used. 


# Constraints on position in the form [a0 a1] * [x; y] <= b
# All three arrays must be of the same length.
# Arrays can be empty.
float64[] spatial_coef_a0
float64[] spatial_coef_a1
float64[] spatial_coef_b

# Two numbers: minimal and maximal bounds // or an empty array
float64[] bounds_orientation

# Two numbers: minimal and maximal bounds // or an empty array
float64[] bounds_steering_velocity

# Two numbers: minimal and maximal bounds // or an empty array
float64[] bounds_tangential_velocity

# Two numbers: minimal and maximal bounds // or an empty array
float64[] bounds_tangential_acceleration

# One number or an empty array
float64[] max_centripetal_acceleration
