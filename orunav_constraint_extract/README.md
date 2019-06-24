Visualize the extractor...




Test examples...

Running the collision check.

roslaunch orunav_constraint_extract grid_map_collision_test.launch 
rosrun orunav_constraint_extract grid_map_collision_test 

Check other test by altering the 'debug_nb' flag.

- Collision check with moving robot
rosrun orunav_constraint_extract grid_map_collision_test --debug_nb 1

- Collision check by varying the orientation, shows valid state space in x,y as free cells in the occupancy map
rosrun orunav_constraint_extract grid_map_collision_test --debug_nb 3

- Collision check with robot checking the footprint vs. the grid.
rosrun orunav_constraint_extract grid_map_collision_test --debug_nb 4




roslaunch polygonfootprint_test.launch 
rosrun orunav_constraint_extract robot_model2d_test


