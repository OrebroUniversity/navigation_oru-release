# If no operation should be done
int32 NO_OPERATION = 1

# If cargo should be unloaded (e.g. lowering the forks)
int32 UNLOAD = 2

# If cargo should be loadeded (e.g. lifting the forks)
int32 LOAD = 3

# If the cargo location should be detected using perception
int32 LOAD_DETECT = 4

# If the fork should be used to lower the support legs
int32 ACTIVATE_SUPPORT_LEGS = 5

# Not sure what this means - TODO
int32 LOAD_DETECT_ACTIVE = 6

# Pick all items in the mission specification
int32 PICK_ITEMS = 7

# Unwrap a pallet
int32 UNWRAP_PALLET = 8

int32 operation
int32 property1
int32 property2
orunav_msgs/IliadItemArray itemlist