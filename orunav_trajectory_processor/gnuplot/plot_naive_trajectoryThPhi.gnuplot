#set terminal png transparent nocrop enhanced font arial 8 size 420,320 
#set output 'hydrocarbonate_level.png'

set title "Theta - Phi plot"

set xlabel "Steps"
set ylabel "Theta / Phi"
#set pointsize 0.1

plot 'naive_traj_output.txt' using 3 with lines t 'naive output - Theta',\
     'naive_traj_output.txt' using 4 with lines t 'naive output - Phi',\
     'naive_traj_path.txt' using 3 with lines t 'input path - Theta',\
     'naive_traj_path.txt' using 4 with lines t 'input path - Phi',\
     'naive_traj_ds.txt' using 3 with lines t 'input ds - Theta',\
     'naive_traj_ds.txt' using 4 with lines t 'input ds - Phi'


# wait for return to be pressed
pause -1
