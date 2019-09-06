#!/usr/bin/env python

from socket import *
import thread
import rospy
import string
import json
import time
import errno
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf
from sbs_motion.srv import *
import subprocess

CRLF = "\n"
sockets=[]
closing = False


def send(req):
    to_send = "{\"send\" : true, \"x\" : \" 0\", \"y\" : \" 0\"}"
    broadcast(to_send)
    subprocess.Popen("rosservice call /sendGoals".split())
    return sendGoalsResponse()

def broadcast(to_send):
    global sockets
    to_send += CRLF
    #rospy.logerr("%s", to_send)
    for s in sockets:
        try:
            #print to_send
            s.send(to_send)
        except IOError:
            rospy.logerr("could not send pose message")
            continue
            
def pose_callback(msg):
    global pub
    to_send = "{\"send\" : false" \
    + ", \"x\" : \"" + str(msg.pose.position.x) \
    + "\", \"y\" : \"" + str(msg.pose.position.y) \
    + "\"}"
    print to_send
    broadcast(to_send)
    pub.publish(msg)

def handler(clientsocket, clientaddr):
    global sockets
    global closing
    
    sockets += [clientsocket]
    clientsocket.settimeout(None)
    
    recv_buffer = 4096
    while not closing:
        try:
            data = clientsocket.recv(recv_buffer)
            if not data:
                break
        except IOError:
            rospy.logerr("error while reading from socket socket")
            break
    clientsocket.close()
    sockets.remove(clientsocket);
    rospy.loginfo("points socket closed")

def run():

    global closing
    global pub
    rospy.init_node("point_server", anonymous=True)
    port = int(rospy.get_param("~port", 17778))
    if port == -1:
        rospy.loginfo("invalid port")
        return

    addr = ("0.0.0.0", port)
    serversocket = socket(AF_INET, SOCK_STREAM)
    serversocket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    serversocket.bind(addr)
    serversocket.listen(2)
    
 
    
    pub =  rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, pose_callback)
    rospy.Service('/shareGoals', sendGoals, send)
    while not rospy.is_shutdown():
        rospy.loginfo("Server is listening for connections on port %d", port)
        try:
            clientsocket, clientaddr = serversocket.accept()
        except IOError as e:
            if e.errno != errno.EINTR:
                raise
            else:
                closing = True
                #rospy.logerr("Ctrl-C?????")

        if not closing:
            rospy.loginfo("Accepted connection from: %s", str(clientaddr))
            thread.start_new_thread(handler, (clientsocket, clientaddr))

    rospy.loginfo("Points server is closing...")
    serversocket.close()
    for clientsocket in sockets:
        clientsocket.close()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException: 
      pass


