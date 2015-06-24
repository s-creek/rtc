#!/usr/bin/gnuplot -persist
set key left
plot   "./log.dat" using 1 w l title "linear"
replot "./log.dat" using 2 w l title "cubic" 
replot "./log.dat" using 3 w l title "quintic"
replot "./log.dat" using 4 w l title "4-1-4"
