#!/usr/bin/env python
import rospy
import socket, string
import json
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseStamped

CRLF = "\n"
sock = None
is_dead = False

def init():
    global sock
    hostname = rospy.get_param("~hostname", "localhost")
    port = int(rospy.get_param("~port", "17777"))
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
    odom_data = json.loads(line)
    if not odom_data:
        return None

    msg = Odometry()
    msg.header.stamp.secs = int(odom_data["secs"])
    msg.header.stamp.nsecs = int(odom_data["tf_nsecs"])
    msg.header.frame_id = "map"
    msg.child_frame_id = "map"
    msg.twist.twist.linear.x = float(odom_data["v"])
    msg.twist.twist.angular.z = float(odom_data["w"])
    msg.pose.pose.position = Point(float(odom_data["px"]), float(odom_data["py"]), float(odom_data["pz"]))
    msg.pose.pose.orientation = Quaternion(float(odom_data["ox"]), float(odom_data["oy"]), float(odom_data["oz"]), float(odom_data["ow"]))
    pose=PoseStamped()
    pose.header=msg.header
    pose.pose=msg.pose.pose
    return msg,pose
    
def run():
    global sock
    global is_dead
    
    rospy.init_node('odom_in_map_client', anonymous=True)
    success = False
    pub = rospy.Publisher(rospy.get_param("~odom_topic", "odom"), Odometry, queue_size=10)
    posePub =  rospy.Publisher(rospy.get_param("~pose_topic", "filtered_ndt_current_pose"), PoseStamped, queue_size=10)
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
                    break
                for line in readlines(sock):
                    line = line.strip()
                    if not line: continue
                    odom,pose = parse_line(line)
                    if not odom == None:
                        pub.publish(odom)
                    if not pose ==None:
                        posePub.publish(pose)
        if not sock is None and not is_dead:
            rospy.loginfo('closing socket...') 
            sock.shutdown(1)
    
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass


