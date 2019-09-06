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

CRLF = "\n"
sockets=[]
closing = False


#current_milli_time = lambda: int(round(time.time() * 1000))

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
    global v,w
   
    to_send = "{\"secs\" : \"" + str(msg.header.stamp.secs) \
    + "\", \"tf_nsecs\" : \"" + str(msg.header.stamp.nsecs) \
    + "\", \"px\" : \"" + str(msg.pose.position.x) \
    + "\", \"py\" : \"" + str(msg.pose.position.y) \
    + "\", \"pz\" : \"" + str(msg.pose.position.z) \
    + "\", \"ox\" : \"" + str(msg.pose.orientation.x) \
    + "\", \"oy\" : \"" + str(msg.pose.orientation.y) \
    + "\", \"oz\" : \"" + str(msg.pose.orientation.z) \
    + "\", \"ow\" : \"" + str(msg.pose.orientation.w) \
    + "\", \"v\" : \"" + v \
    + "\", \"w\" : \"" + w\
    + "\"}"
    #print to_send
    broadcast(to_send)
    
def odometry_callback(msg):
    global v,w
   
    v= str(msg.twist.twist.linear.x) 
    w= str(msg.twist.twist.angular.z) 

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
            rospy.logerr("error while reading from pose socket")
            break
    clientsocket.close()
    sockets.remove(clientsocket);
    rospy.loginfo("pose socket closed")

def run():

    global closing
    global v,w
   
    v=w="0"
    rospy.init_node("pose_in_map_server", anonymous=True)
    port = int(rospy.get_param("~port", 17777))
    if port == -1:
        rospy.loginfo("invalid port")
        return

    addr = ("0.0.0.0", port)
    serversocket = socket(AF_INET, SOCK_STREAM)
    serversocket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    serversocket.bind(addr)
    serversocket.listen(2)
    
 

    
    rospy.Subscriber(rospy.get_param("~pose_topic", "filtered_ndt_current_pose"), PoseStamped, pose_callback)
    rospy.Subscriber(rospy.get_param("~odom_topic", "odom"), Odometry, odometry_callback)
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

    rospy.loginfo("Pose server is closing...")
    serversocket.close()
    for clientsocket in sockets:
        clientsocket.close()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException: 
      pass


