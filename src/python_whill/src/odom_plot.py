#!/usr/bin/env python

#Created on Wednesday September 21th 2018
#author: Koya Hashizume

import rospy
from nav_msgs.msg import Odometry

def odom_write_callback(odom):

    string = [str(odom.pose.pose.position.x) ," " ,str(odom.pose.pose.position.y) ,'\n']

    
    f.writelines(string)
    f.flush()

    return


def name():

    rospy.init_node('odom_ploter',anonymous=True)
#    path = 'home/hashizume/txt/odom.dat'
    global f
    f = open('/home/yoichims/txt/odom.dat',mode='a')

    rospy.Subscriber("/odom",Odometry,odom_write_callback)


    rospy.spin()
    f.close()
if __name__ == "__main__":
    name()
