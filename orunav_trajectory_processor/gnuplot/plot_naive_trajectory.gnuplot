#set terminal png transparent nocrop enhanced font arial 8 size 420,320 
#set output 'hydrocarbonate_level.png'

set title "Naive Trajectory"
set xlabel "Time"
set ylabel ""
set pointsize 0.1
#set yrange[-1.5:1.5]


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


# plot 'naive_trajectory.txt' using 2:3 with lines t 'dt',\
#      'naive_trajectory.txt' using 2:4 with lines t 'v_step',\
#      'naive_trajectory.txt' using 2:5 with lines t 'w_step',\
#      'naive_trajectory.txt' using 2:6 with lp pt 2 ps 4 t 'dd',\
#      'naive_trajectory.txt' using 2:7 with lp pt 2 ps 4 t 'dphi',\
#      'naive_trajectory.txt' using 1:8 with points ps 2 pt 3 t 'cc_point.valid',\
#      'naive_trajectory.txt' using 1:9 with lp ps 2 pt 3 t 'cc_point.v',\
#      'naive_trajectory.txt' using 1:10 with lp ps 2 pt 3 t 'cc_point.w',\
#      'naive_trajectory.txt' using 1:11 with lp ps 2 pt 3 t 'c_point.v',\
#      'naive_trajectory.txt' using 1:12 with lp ps 2 pt 3 t 'c_point.w',\
#      'naive_trajectory.txt' using 2:13 with lp t 'acc',\
#      'naive_trajectory.txt' using 2:14 with lp t 'dir',\
#      'naive_trajectory.txt' using 2:15 with lp t 'phi',\
#      'naive_trajectory.txt' using 2:16 with lp t 'phi_step',\
#      'naive_trajectory_fixed_dt.txt' using 2:3 with lp ps 2 pt 1 t 'fixed_dt',\
#      'naive_trajectory_fixed_dt.txt' using 2:4 with lp ps 2 pt 1 t 'fixed_v_step',\
#      'naive_trajectory_fixed_dt.txt' using 2:5 with lp ps 2 pt 1 t 'fixed_w_step',\
#      'naive_trajectory_fixed_dt.txt' using 2:6 with lp pt 2 ps 4 t 'fixed_dd',\
#      'naive_trajectory_fixed_dt.txt' using 2:7 with lp pt 2 ps 4 t 'fixed_dphi',\
#      'naive_trajectory_fixed_dt.txt' using 2:15 with lp t 'phi',\
#      'naive_trajectory_fixed_dt.txt' using 2:16 with lp t 'phi_step'

# plot 'naive_trajectory.txt' using 2:3 with lines t 'dt',\
#      'naive_trajectory.txt' using 2:4 with lines t 'v_step',\
#      'naive_trajectory.txt' using 2:5 with lines t 'w_step',\
#      'naive_trajectory.txt' using 2:6 with lp pt 2 ps 4 t 'dd',\
#      'naive_trajectory.txt' using 2:7 with lp pt 2 ps 4 t 'dphi',\
#      'naive_trajectory.txt' using 2:14 with lp t 'dir',\
#      'naive_trajectory.txt' using 1:15 with lp t 'phi',\
#      'naive_trajectory.txt' using 2:16 with lp t 'phi_step',\
#      'naive_trajectory.txt' using 1:17 with lp t 'x',\
#      'naive_trajectory.txt' using 2:18 with lp t 'x_step',\
#      'naive_trajectory_fixed_dt.txt' using 1:15 with lp t 'fixed_phi',\
#      'naive_trajectory_fixed_dt.txt' using 2:16 with lp t 'fixed_phi_step',\
#      'naive_trajectory_fixed_dt.txt' using 1:17 with lp t 'fixed_x',\
#      'naive_trajectory_fixed_dt.txt' using 2:18 with lp t 'fixed_x_step'

# plot 'naive_trajectory.txt' using 2:7 with lp t 'dphi',\
#      'naive_trajectory.txt' using 1:11 with lp ps 2 pt 3 t 'c_point.v',\
#      'naive_trajectory.txt' using 1:12 with lp ps 2 pt 3 t 'c_point.w',\
#      'naive_trajectory.txt' using 2:14 with lp t 'dir',\
#      'naive_trajectory.txt' using 1:15 with lp t 'phi',\
#      'naive_trajectory.txt' using 2:16 with lp t 'phi_step/dt',\
#      'naive_trajectory.txt' using 2:18 with lp t 'x_step/dt',\
#      'naive_trajectory.txt' using 2:4 with lines t 'v_step',\
#      'naive_trajectory.txt' using 2:5 with lines t 'w_step',\
#      'naive_trajectory_fixed_dt.txt' using 2:7 with lp t 'fixed_dphi',\
#      'naive_trajectory_fixed_dt.txt' using 1:15 with lp t 'fixed_phi',\
#      'naive_trajectory_fixed_dt.txt' using 2:16 with lp t 'fixed_phi_step/dt',\
#      'naive_trajectory_fixed_dt.txt' using 2:18 with lp t 'fixed_x_step/dt',\
#      'naive_trajectory.txt' using 1:17 with lp t 'x',\
#      'naive_trajectory_fixed_dt.txt' using 1:17 with lp t 'fixed_x',\
#      'naive_trajectory_fixed_dt.txt' using 2:4 with lp ps 2 pt 1 t 'fixed_v_step',\
#      'naive_trajectory_fixed_dt.txt' using 2:5 with lp ps 2 pt 1 t 'fixed_w_step'

plot 'naive_trajectory.txt' using 2:14 with lp t 'dir',\
    'naive_trajectory.txt' using 2:4 with lines t 'v_step',\
    'naive_trajectory.txt' using 2:5 with lines t 'w_step',\
    'naive_trajectory.txt' using ($19-$20)>=0?($19-$20):1/0:1 with p ps 3 t 'coord times',\
    'naive_trajectory_fixed_dt.txt' using 2:4 with lp ps 2 pt 1 t 'fixed_v_step',\
    'naive_trajectory_fixed_dt.txt' using 2:5 with lp ps 2 pt 1 t 'fixed_w_step',\
    'naive_trajectory.txt' using 1:1 with lp t 'no - coord times (fastest speed)',\
    '< paste output.traj naive_trajectory_fixed_dt.txt' using 7:5 with lp t 'output traj (v)',\
    '< paste output.traj naive_trajectory_fixed_dt.txt' using 7:6 with lp t 'output traj (w)'


#    'naive_trajectory.txt' using ($19-$20)>=0?($19-$20):1/0:1 with p ps 3 t 'coord times',\
#    'naive_trajectory.txt' using 1:($19-$20)>=0?($19-$20):1/0 with p ps 3 t 'coord times',\



# wait for return to be pressed
pause -1
