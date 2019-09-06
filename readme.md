To run:
run start_vehicle1.py partner_host:=[other computer's hostname or ip]. on first computer
run start_vehicle2.py partner_host:=[other computer's hostname or ip]. on second computer
run send_sample_points.py on either computer (or place the subgoals by hand in Rviz using 2d Nav goal on rosservice call /shareGoals on either computer)

for simulation on 2 computers, do as above, but include use_simulator:=true
For simulation on a single computer, launch both.launch instead

Required topics from Robot:
/<namespace>/filtered_ndt_current_pose
/<namespace>/odom

https://docs.google.com/presentation/d/1JlsPfJ8eXRXH4gGB4AIRWnJIowUb2J2I6TN00jHAV4I/edit?usp=sharing

