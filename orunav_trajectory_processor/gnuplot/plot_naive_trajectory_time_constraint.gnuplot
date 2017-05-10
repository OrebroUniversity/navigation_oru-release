#set terminal png transparent nocrop enhanced font arial 8 size 420,320 
#set output 'hydrocarbonate_level.png'

set title "Naive Trajectory"
set xlabel "Time"
set ylabel ""
set pointsize 0.1
#set yrange[-1.5:1.5]


plot 'naive_trajectory.txt' using 2:14 with lp t 'dir',\
    'naive_trajectory.txt' using 2:4 with lines t 'v_step',\
    'naive_trajectory.txt' using 2:5 with lines t 'w_step',\
    'naive_trajectory.txt' using 2:19 with p t 'coord times'



# wait for return to be pressed
pause -1
