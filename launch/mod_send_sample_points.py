#! /usr/bin/env python

points=[(32.555153,0.122802),
        #(33.004860,-0.345771),
        (33.027851,-0.937004),
        #(33.171577,-0.505785),
        (32.884113,-1.655695),
        #(21.948320,-16.946842),
        (22.28320,-16.946842),
        (20.982603,-17.179396),
        (19.971172,-16.932552),
        (14.333725,-13.333253),
        (12.732042,-7.002749),
        (12.753303,10.522362)
]
if __name__ == '__main__':
    import subprocess
    import time
    try:
        from subprocess import DEVNULL # py3k
    except ImportError:
        import os
        DEVNULL = open(os.devnull, 'wb')

    command="rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped 'header: \n  seq: {} \n  stamp: \n    secs: 0\n    nsecs:         0\n  frame_id: map\npose: \n    position: \n      x: {}\n      y: {}\n      z: 0.0\n    orientation: \n      x: 0.0\n      y: 0.0\n      z: 0.0\n      w: 1.0'"
    for count,(x,y) in enumerate(points):
        print ("Publishing point {}\n\t x:{} \t y:{}".format(count+1,x,y))
        p=subprocess.Popen(command.format(count,x,y),shell=True,stdout=DEVNULL)
        time.sleep(0.2)
        p.terminate()
    print("Sending points to agents")
    time.sleep(0.5)
    p=subprocess.Popen("rosservice call /shareGoals", shell=True,stdout=DEVNULL)
    time.sleep(0.5)
    p.terminate()
