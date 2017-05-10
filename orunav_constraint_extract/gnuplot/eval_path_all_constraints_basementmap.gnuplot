reset

set terminal postscript eps size 3.5,2.62 enhanced color font 'Helvetica,12' lw 1
set output 'eval_path_all_constraints_basementmap.eps'


set title "Position and angular constraints"
set xlabel "x (m)"
set ylabel "y (m)"
set pointsize 0.5
set key top right 
#bottom left

#unset tics

set macro 
    my_res = "10" # Inverted resolution
    my_x_off = "-0.85" # offset in x-dir
    my_y_off = "-0.9" # offet in y-dir


set xtics ('0' 0, '2' 20, '4' 40, '6' 60, '8' 80, '10' 100, '12' 120, '14' 140, '16' 160)
set ytics ('0' 0, '2' 20, '4' 40, '6' 60, '8' 80, '10' 100, '12' 120, '14' 140, '16' 160)
set tics scale 0.75

set size ratio -1
set xrange [40:120]
set yrange [20:80]

#set style line 1 linecolor rgb '#0060ad' linetype 1 linewidth 1 # blue
#set style line 2 linecolor rgb '#bb181f' linetype 1 linewidth 1 # red
#set style line 3 linecolor rgb '#44bb18' linetype 2 linewidth 2 # green

    set macro
my_line_width = "2"

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



set style line 1  linecolor rgbcolor blue_050  linewidth @my_line_width pt 7 lt 1 ps 2
set style line 2  linecolor rgbcolor green_050 linewidth 1 pt 5 lt 1 ps 2
set style line 3  linecolor rgbcolor red_050   linewidth 1 pt 9 lt 1 ps 2
set style line 4  linecolor rgbcolor brown_050 linewidth @my_line_width pt 13 lt 1 ps 2
set style line 5  linecolor rgbcolor blue_075  linewidth 1 pt 7 lt 1 ps 2
set style line 6  linecolor rgbcolor brown_025  linewidth 1 pt 7 lt 1 ps 2



set macro
my_nb_skip = "5"


plot 'basement_gray.png' binary filetype=png with rgbimage t '',\
     '~/.ros/polygonconstraint_service_outer_constraint_0-0.dat' every :@my_nb_skip using (($1+@my_x_off)*@my_res):(($2+@my_y_off)*@my_res) with lines ls 6 t 'Constraints outer',\
     '~/.ros/polygonconstraint_service_all_0-0position.constraints.dat' every :@my_nb_skip using (($1+@my_x_off)*@my_res):(($2+@my_y_off)*@my_res) with lines ls 5 t 'Constraints position',\
     '~/.ros/polygonconstraint_service_path_0-0.path' using (($1+@my_x_off)*@my_res):(($2+@my_y_off)*@my_res) with lines ls 1 t 'Original path',\
     '~/.ros/polygonconstraint_service_all_0-0angular.constraints.dat' every @my_nb_skip using (($3+@my_x_off)*@my_res):(($4+@my_y_off)*@my_res):(@my_res*cos($1)):(@my_res*sin($1)) with vectors ls 2 t 'Constraints angular',\
     '~/.ros/polygonconstraint_service_all_0-0angular.constraints.dat' every @my_nb_skip using (($3+@my_x_off)*@my_res):(($4+@my_y_off)*@my_res):(@my_res*cos($2)):(@my_res*sin($2)) with vectors ls 3 t 'Constraints angular'
#pause -1
