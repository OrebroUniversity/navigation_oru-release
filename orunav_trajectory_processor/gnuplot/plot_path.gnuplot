#set terminal png transparent nocrop enhanced font arial 8 size 420,320 
#set output 'path.png'

set title "X - Y plot"


set xlabel "Pos x"
set ylabel "Pos y"

plot 'debug_me.path' using 1:2 with points t 'xy'
#plot 'debug_me.path' using 1 with points t 'x',\
#     'debug_me.path' using 2 with points t 'y'

# wait for return to be pressed
pause -1
