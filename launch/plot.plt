set term X11
set size square

plot 'xpair_log.txt' u 1:5 w lp,\
     'xpair_log.txt' u 1:7 w lp,\
      'pair_log.txt' u 1:11 w lp,\
     'pair_log.txt' u 1:13 w lp
     pause -1


set size ratio -1



#set datafile separator ", " 
#my_command='< sed -n 1,23p pass.txt'
#interval = 20
#set palette model RGB

#set palette rgb 15,5, 7, 7,5,15

#set palette defined ( 0 "yellow", 1 "blue", 2 "green", 3 "orange", 4 "black", 5 "red", 6 "yellow")
#, 7 "orange")


#set palette defined ( 1 "green",  2 "green", 3 "blue", 4 "black", 5 "red", 6 "orange", 7 "yellow")
#set palette functions sqrt(gray), gray**3, sin(gray*2*pi)

set palette defined ( 0 '#000090',1 '#000fff',2 '#0090ff',3 '#0fffee',4 '#90ff70',5 '#ffee00',6 '#ff7000',7 '#ee0000',8 '#7f0000', 9 '#50000')

set palette defined ( 0 '#000050', 1 '#000090',2 '#000fff',3 '#0090ff',4 '#0fffee',5 '#90ff70',6 '#ffee00',7 '#ff7000',8 '#ee0000',9 '#7f0000', 10 '#50000')





#set palette rgb 7,5,15; set title "traditional pm3d\n(black-blue-red-yellow)"
#splot g(x)
###set palette defined rgb ("black-blue-red-yellow")
#set palette model RGB rgbformulae 4,7,7,5,15



set cbrange [-1.35:1.35]
#set cbrange [-2:2]


FILE='log_2018-06-13-16-44-18.txt'


plot 'files/7th_floor.map' u 6:8:10 w d t '' ,\
     'pair_log.txt' u 2:3:5 w lp palette t '',\
     'pair_log.txt' u 7:8:($10*-1) w lp palette t ''

pause -1
set palette

plot 'files/map.map' u 6:8:10 w d t '' ,\
     'pair_log.txt' u 2:3:5 w lp palette t ''

replot
pause -1


plot 'map.map' u 6:8:10 w d t '' ,\
     'clean_only_whill_output.txt' u 1:2:5 w lp palette t ''
pause -1





plot 'map.map' u 6:8:10 w d t '' ,\
     'test_output.txt' u 1:2:17 w lp palette t ''
pause -1


plot 'map.map' u 6:8:10 w d t '' ,\
     'test.txt' u 2:3:5 w lp palette t '',\
     'test.txt' u 7:8:10 w lp palette t ''
pause -1

plot 'map.map' u 6:8:10 w d t '' ,\
     'test_output.txt' u 1:2:5 w l palette t '',\
     'test2_output.txt' u 1:2:5 w l palette t ''
pause -1



plot 'map.map' u 6:8:10 w d t '' ,\
     FILE u 2:3:5 w l palette t '',\
     FILE u 7:8:10 w l palette t ''
pause -1

plot 'map.map' u 6:8:10 w d t '' ,\
     'log_2018-06-13-20-17-37.txt' u 2:3:5 w l palette t '',\
     'log_2018-06-13-20-17-37.txt' u 7:8:10 w l palette t ''
pause -1




plot 'robot_spline.txt' u 2:3:6 w lp palette t '',\
      '../python_loader/map.csv' u 6:8 w d t '' ls 18
pause -1

set xrange[-150:40]
set yrange[-60:50]


#range= "1,83p"

my_command='< sed -n 1,23p pass.txt'
my_command='< sed -n 50,150p pass.txt'

#set title 'Driving through pedestrians'
#set title 'Driving through pedestrians\n Facing other pedestrians'


#plot 'map.csv' u 6:8 
#pause -1


pl my_command u 2:3 w lp t '' ls 1  pointinterval 4,\
      '' using 2:3:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 4:5 w lp t '' ls 2  pointinterval 4,\
      '' using 4:5:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 6:7 w lp t '' ls 3  pointinterval 4,\
      '' using 6:7:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 8:9 w lp t '' ls 4  pointinterval 4,\
      '' using 8:9:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      'map.csv' u 6:8 w d t '' ls 18
#                  set term svg
 #     set output 'social_passing_by_concept1.svg'
 #     replot
      pause -1


set title 'Driving through pedestrians\n Starting the interaction'

my_command='< sed -n 1,50p pass.txt'

pl my_command u 2:3 w lp t '' ls 1  pointinterval 4,\
      '' using 2:3:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 4:5 w lp t '' ls 2  pointinterval 4,\
      '' using 4:5:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 6:7 w lp t '' ls 3  pointinterval 4,\
      '' using 6:7:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 8:9 w lp t '' ls 4  pointinterval 4,\
      '' using 8:9:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      'map.csv' u 6:8 w d t '' ls 18
      
      #set term svg
      #set output 'social_passing_by_concept2.svg'
      #replot
pause -1

       



my_command='< sed -n 21,63p pass.txt'

pl my_command u 2:3 w lp t '' ls 1  pointinterval 4,\
      '' using 2:3:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 4:5 w lp t '' ls 2  pointinterval 4,\
      '' using 4:5:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 6:7 w lp t '' ls 3  pointinterval 4,\
      '' using 6:7:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 8:9 w lp t '' ls 4  pointinterval 4,\
      '' using 8:9:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      'map.csv' u 6:8 w d t '' ls 18
      

pause -1


my_command='< sed -n 21,82p pass.txt'

pl my_command u 2:3 w lp t '' ls 1  pointinterval 2,\
      '' using 2:3:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 4:5 w lp t '' ls 2  pointinterval 2,\
      '' using 4:5:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 6:7 w lp t '' ls 3  pointinterval 2,\
      '' using 6:7:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 8:9 w lp t '' ls 4  pointinterval 2,\
      '' using 8:9:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      'map.csv' u 6:8 w d t '' ls 18
#            set term svg
#      set output 'social_passing_by_concept3_5.svg'
#      replot
pause -1



my_command='< sed -n 21,102p pass.txt'

pl my_command u 2:3 w lp t '' ls 1  pointinterval 2,\
      '' using 2:3:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 4:5 w lp t '' ls 2  pointinterval 2,\
      '' using 4:5:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 6:7 w lp t '' ls 3  pointinterval 2,\
      '' using 6:7:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 8:9 w lp t '' ls 4  pointinterval 2,\
      '' using 8:9:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      'map.csv' u 6:8 w d t '' ls 18
#            set term svg
#      set output 'social_passing_by_concept4.svg'
#      replot
pause -1


my_command='< sed -n 50,150p pass.txt'

pl my_command u 2:3 w lp t '' ls 6  pointinterval 2,\
      '' using 2:3:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 4:5 w lp t '' ls 5  pointinterval 2,\
      '' using 4:5:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 6:7 w lp t '' ls 1  pointinterval 2,\
      '' using 6:7:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      my_command u 8:9 w lp t '' ls 2  pointinterval 2,\
      '' using 8:9:($1-151183777839) every interval  with labels offset 0,0 notitle
pause -1















pl '< sed -n 1,50p pass.txt' u 2:3 w lp t '' ls 6  pointinterval 7,\
      '' using 2:3:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      '< sed -n 1,50p pass.txt' u 4:5 w lp t '' ls 5  pointinterval 7,\
      '' using 4:5:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      '< sed -n 1,50p pass.txt' u 6:7 w lp t '' ls 1  pointinterval 7,\
      '' using 6:7:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      '< sed -n 1,50p pass.txt' u 8:9 w lp t '' ls 2  pointinterval 7,\
      '' using 8:9:($1-151183777839) every interval  with labels offset 0,0 notitle
pause -1


pl '< sed -n 1,75p pass.txt' u 2:3 w lp t '' ls 6  pointinterval 7,\
      '' using 2:3:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      '< sed -n 1,25p pass.txt' u 4:5 w lp t '' ls 5  pointinterval 7,\
      '' using 4:5:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      '< sed -n 1,25p pass.txt' u 6:7 w lp t '' ls 1  pointinterval 7,\
      '' using 6:7:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      '< sed -n 1,25p pass.txt' u 8:9 w lp t '' ls 2  pointinterval 7,\
      '' using 8:9:($1-151183777839) every interval  with labels offset 0,0 notitle
pause -1



pl '< sed -n . range . pass.txt' u 2:3 w lp t '' ls 6  pointinterval 7,\
      '' using 2:3:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      '< sed -n 1,25p pass.txt' u 4:5 w lp t '' ls 5  pointinterval 7,\
      '' using 4:5:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      '< sed -n 1,25p pass.txt' u 6:7 w lp t '' ls 1  pointinterval 7,\
      '' using 6:7:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      '< sed -n 1,25p pass.txt' u 8:9 w lp t '' ls 2  pointinterval 7,\
      '' using 8:9:($1-151183777839) every interval  with labels offset 0,0 notitle
pause -1





pl '< head -100 log_2017-11-28-11-54-25_pass1.txt' u 2:3 w lp t '' ls 6  pointinterval 7
pause -1



plot 'x1m_res.pcd' u ($1*1):($2*1) w d,\
     'pass1.txt' u 2:3 w lp t '' ls 6  pointinterval 7,\
      '' using 2:3:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      'pass1.txt' u 4:5 w lp t '' ls 5  pointinterval 7,\
      '' using 4:5:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      'pass1.txt' u 6:7 w lp t '' ls 1  pointinterval 7,\
      '' using 6:7:($1-151183777839) every interval  with labels offset 0,0 notitle,\
      'pass1.txt' u 8:9 w lp t '' ls 2  pointinterval 7,\
      '' using 8:9:($1-151183777839) every interval  with labels offset 0,0 notitle

pause -1

set term png
set output 'collaborative_walking5.png'
replot
pause -1

