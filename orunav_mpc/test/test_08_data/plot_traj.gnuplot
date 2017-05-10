#set terminal png transparent nocrop enhanced font arial 8 size 420,320 
#set output 'hydrocarbonate_level.png'

set title "Trajectory MPC -smoothing comparision"
set xlabel "Time"
set ylabel ""
set pointsize 0.1

plot 'original.traj' using 5 with lp t 'Orig: v',\
      'mpc_smooth.traj' using 5 with p t 'MPC: v'


# wait for return to be pressed
pause -1
