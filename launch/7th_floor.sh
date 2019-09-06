#! /bin/bash
source /opt/ros/kinetic/setup.bash 
source /home/yoichims/Dropbox/Research/whill_side_by_side/devel/setup.bash
       


gnome-terminal -e "roslaunch /home/yoichims/Dropbox/Research/whill_side_by_side/launch/7th_floor.launch"&

sleep 5

/home/yoichims/Dropbox/Research/whill_side_by_side/centerlines/send_composite_cw.sh


