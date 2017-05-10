#set terminal png transparent nocrop enhanced font arial 8 size 420,320 
#set output 'hydrocarbonate_level.png'

set title "Trajectory MPC -smoothing comparision"
set xlabel "Time"
set ylabel ""
set pointsize 0.1

plot 'original.traj' using 1:2 with p t 'Original',\
     'mpc_smooth.traj' using 1:2 with p t 'MPC-smooth'



# wait for return to be pressed
pause -1
