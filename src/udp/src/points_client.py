#!/usr/bin/env python
import rospy
import socket, string
import json
import time
import subprocess

from geometry_msgs.msg import Point, Quaternion, PoseStamped
from sbs_motion.srv import *


CRLF = "\n"
sock = None
is_dead = False

def init():
    global sock
    hostname = rospy.get_param("~hostname", "localhost")
    port = int(rospy.get_param("~port", "17778"))
    for res in socket.getaddrinfo(hostname, port, socket.AF_INET, socket.SOCK_STREAM):
        af, socktype, proto, canonname, sa = res
        try:
            sock = socket.socket(af, socktype, proto)
        except IOError:
            sock = None
            continue
        try:
            sock.connect(sa)
        except IOError:
            sock.close()
            sock = None
            continue
        break
    if sock is None:
        rospy.logerr("could not open socket to %s:%d", hostname, port)
        return False
    return True

def readlines(sock, recv_buffer = 4096, delim = CRLF):
    global is_dead
    buffer = ''
    data = True
    while data:
        try:
            data = sock.recv(recv_buffer)
            buffer += data
            #print("data: " + data)
            while buffer.find(delim) != -1:
                line, buffer = buffer.split(delim, 1)
                yield line
        except IOError:
            rospy.logerr("IOError: could not read lines")
            is_dead = True
            break
    # this should be added on all socket client
    if not is_dead:
        rospy.logerr("Could no longer read data, the server probably went down")
        is_dead = True
    return 

def parse_line(line):
    point_data = json.loads(line)
    if not point_data:
        return None
    
    msg=PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position=Point(float(point_data["x"]), float(point_data["y"]), 0)
    send=point_data["send"]
    return msg,send

def run():
    global sock 
    global is_dead

    rospy.init_node('point_client', anonymous=True)
    success = False
    pub =  rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
   
    while not rospy.is_shutdown():
        while not rospy.is_shutdown() and not success:
            success = init();
            if success:
                is_dead=False
                break
            else:
                time.sleep(3.0)
   

        if not rospy.is_shutdown() and success :
              while not rospy.is_shutdown():
                if is_dead :
                    success=False
                    count=0
                    break
                for line in readlines(sock):
                    line = line.strip()
                    if not line: continue
                    point,send = parse_line(line)
                    if not point ==None and not send:
                        pub.publish(point)
                    if send:
                        subprocess.Popen("rosservice call /sendGoals".split())
                            
        if not sock is None and not is_dead:
            rospy.loginfo('closing socket...') 
            sock.shutdown(1)
    
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass


