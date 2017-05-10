reset

# eps
#set terminal postscript eps size 3.5,2.62 enhanced color font 'Helvetica,20' lw 2
#set output 'plot_naive_trajectory_no_dt.eps'

set title "Trajectory Generation"
set xlabel "Steps"
set pointsize 0.1
set yrange[-1.5:1.5]

   blue_000 = "#A9BDE6" # = rgb(169,189,230)
   blue_025 = "#7297E6" # = rgb(114,151,230)
   blue_050 = "#1D4599" # = rgb(29,69,153)
   blue_075 = "#2F3F60" # = rgb(47,63,96)
   blue_100 = "#031A49" # = rgb(3,26,73)

   green_000 = "#A6EBB5" # = rgb(166,235,181)
   green_025 = "#67EB84" # = rgb(103,235,132)
   green_050 = "#11AD34" # = rgb(17,173,52)
   green_075 = "#2F6C3D" # = rgb(47,108,61)
   green_100 = "#025214" # = rgb(2,82,20)

   red_000 = "#F9B7B0" # = rgb(249,183,176)
   red_025 = "#F97A6D" # = rgb(249,122,109)
   red_050 = "#E62B17" # = rgb(230,43,23)
   red_075 = "#8F463F" # = rgb(143,70,63)
   red_100 = "#6D0D03" # = rgb(109,13,3)

   brown_000 = "#F9E0B0" # = rgb(249,224,176)
   brown_025 = "#F9C96D" # = rgb(249,201,109)
   brown_050 = "#E69F17" # = rgb(230,159,23)
   brown_075 = "#8F743F" # = rgb(143,116,63)
   brown_100 = "#6D4903" # = rgb(109,73,3)

set style line 1  linecolor rgbcolor red_050   linewidth 2
set style line 2  linecolor rgbcolor green_050 linewidth 2
set style line 3  linecolor rgbcolor blue_050  linewidth 2


plot 'naive_trajectory.txt' using 1:7 with lines ls 1 t 'v (m/s)',\
     'naive_trajectory.txt' using 1:8 with lines ls 2 t '{/Symbol w} (rad/s)',\
     'naive_trajectory.txt' using 1:(-$9) with lines ls 3 t 'a (m/s^2)'
     

# wait for return to be pressed
pause -1
