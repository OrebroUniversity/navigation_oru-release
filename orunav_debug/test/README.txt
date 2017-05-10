Visualize the MPC controller output.
This folder contains two files obtained from a test drive with one of the CiTi trucks - controller.log and sensor.log. These files can be parsed and output using

Run in separate teminals
roscore
rosrun rviz rviz # Make sure you subsribe to the visualization_marker topic
rosrun orunav_debugger control_log_parser --sensor_log_filename sensor.log
