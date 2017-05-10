#set terminal png transparent nocrop enhanced font arial 8 size 420,320 
#set output 'hydrocarbonate_level.png'

set title "X - Y plot"


set xlabel "Pos x"
set ylabel "Pos y"
#set pointsize 0.1

#     'naive_trajectory.txt' using 1:3 with lines t 'dx',\
#     'naive_trajectory.txt' using 1:4 with lines t 'dy',\
#     'naive_trajectory.txt' using 1:9 with lines t 'a',\
#     'naive_trajectory.txt' using 1:12 with lines t 'acc2'
#     'naive_trajectory.txt' using 1:10 with lines t 'v2',\
#     'naive_trajectory.txt' using 1:11 with lines t 'dt2'
#     'naive_trajectory.txt' using 1:10 with lines t 'v2',\
#     'naive_trajectory.txt' using 1:12 with lines t 'acc2'
#     'naive_trajectory.txt' using 1:3 with lines t 'dx',\
#     'naive_trajectory.txt' using 1:4 with lines t 'dy',\
#     'naive_trajectory.txt' using 1:5 with lines t 'dth',\
#     'naive_trajectory.txt' using 1:6 with lines t 'dphi',\
#     'naive_trajectory.txt' using 1:12 with lines t 'acc2'

plot 'naive_trajectoryXY.txt' using 1:2 with lines t 'input path',\
     'naive_trajectoryXY.txt' using 1:2 with points pt 7 ps 1.5 t 'input points',\
     'naive_trajectory_fixed_dtXY.txt' using 1:2 with lp t 'naive output fixed dt',\
     'naive_trajectory_fixed_dt_fwd_simXY.txt' using 1:2 with lp t 'output fwd simulation',\
     'naive_trajectoryXY_coordination_times.txt' using 1:2 with lp pt 7 ps 2 t 'coordination points',\
     'output.traj' using 1:2 with lp t 'output traj'

     

#plot 'naive_traj_output.txt' using 1:2 with lines t 'naive output'
#     'naive_traj_path.txt' using 1:2 with points pt 7 ps 0.5 t 'input path',\
#     'naive_traj_ds.txt' using 1:2 with lines t 'input ds',\
#     'naive_traj_output_fixed_dt.txt' using 1:2 with points pt 1 ps 6 t 'naive output fixed dt',\
#     'naive_traj_s_output_fixed_dt.txt' using 1:2 with points pt 1 ps 4 t 'naive output s fixed dt'

# wait for return to be pressed
pause -1
