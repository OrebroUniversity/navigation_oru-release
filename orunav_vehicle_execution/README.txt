To run the vehicle to multiple targets from a predefined file use

> rosrun orunav_vehicle_execution vehicle_execution_client _targets_feil_name_:=targets.dat

in order to run the next task you need to call the /next_task service

> rosservice call /next_task

If you want the system to automatically dispatch the complete targets list automatically specify what index in the target file the robot should start from

> rosrun orunav_vehicle_execution vehicle_execution_client _targets_feil_name_:=targets.dat _task_id:=0

To generate a set of targets a bit randomly you can use:

> rosrun orunav_vehicle_execution generate_random_targets

to generate a targets.dat file.
