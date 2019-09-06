set term X11
set size square
set size ratio -1



#plot 'z7th_floor.map' u 6:8 w d t '' ,\
#     'cw_centerline_composite.txt' u 2:3 w p t 'CW',\
#     'cw_centerline_composite_for_path.txt' u 1:2 w l t ''
#pause -1

#plot 'z7th_floor.map' u 6:8 w d t '' ,\
#     'ccw_centerline_composite.txt' u 2:3 w p t 'CW',\
#     'ccw_centerline_composite_for_path.txt' u 1:2 w l t ''
#pause -1




plot '7th_floor.map' u 6:8 w d t '' ,\
     'cw_centerline_composite.txt' u 2:3 w l t 'CW',\
     'ccw_centerline_composite.txt' u 2:3 w l t 'CCW'
     pause -1


plot '7th_floor.map' u 6:8 w d t '' ,\
     'cw_centerline_composite.txt' u 2:3 w l t 'CW',\
     'xyResamplingList.txt' u 1:2 w lp
     pause -1


set palette defined ( 0 '#000050', 1 '#000090',2 '#000fff',3 '#0090ff',4 '#0fffee',5 '#90ff70',6 '#ffee00',7 '#ff7000',8 '#ee0000',9 '#7f0000', 10 '#50000')


plot '7th_floor.map' u 6:8:10 w d t '' ,\
     'zccw_centerline.txt' u 2:3:4 w p palette t '',\
     'ccw_distribution.txt' u 1:2:3 w p palette t '',\
     'zccw_centerline_output.txt' u 2:3:4 w l palette t '',\
     'ccw_centerline_output_output_output.txt' u 2:3:4 w l palette t ''
pause -1

plot '7th_floor.map' u 6:8:10 w d t '' ,\
     'zcw_centerline.txt' u 2:3:4 w l palette t '',\
     'cw_partial_distribution.txt' u 1:2:3 w p palette t '',\
     'zcw_centerline_output_output.txt' u 2:3:4 w l palette t '',\
     'cw_centerline_output_output_output_output_output.txt' u 2:3:4 w l palette t ''
pause -1
     #'cw_distribution.txt' u 1:2:3 w p palette t '',\

