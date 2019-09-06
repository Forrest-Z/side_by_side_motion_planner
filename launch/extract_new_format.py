#!/usr/bin/env python

# *************************************************************  
# Program to output data for slam6d software
# based on corresponding program in Navigation1.0 project (by Luis et al.)
# Writes: resulting pose and scan data in :
# for6dslamDat/scan*.pose and for6dslamDat/scan*.3d respectively
# *************************************************************

import rospy, tf
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *


import sensor_msgs.point_cloud2 as pc2
import math
import os, shutil, sys

ADJUST_FOR_WHILL_PASSENGER = True
PASSENGER_DIST = 0.17


odomx=0
odomy=0 
odomtheta=0
odomv=0
odomw=0
posex=0
posey=0
posetheta=0

v2_odomx=0
v2_odomy=0 
v2_odomtheta=0
v2_odomv=0
v2_odomw=0

prev_time = 0 #rospy.Time.now()
acc_time =0

cmd_vel_v = 0
cmd_vel_w = 0

v2_cmd_vel_v = 0
v2_cmd_vel_w = 0


def cmd_vel_callback_1 (msg):
    global cmd_vel_v, cmd_vel_w
    cmd_vel_v = msg.linear.x
    cmd_vel_w = msg.angular.z
    #rospy.logerr ("vel %.3f %.3f" % (msg.linear.x , msg.angular.z ))

def cmd_vel_callback_2 (msg):
    global v2_cmd_vel_v, v2_cmd_vel_w
    v2_cmd_vel_v = msg.linear.x
    v2_cmd_vel_w = msg.angular.z
    #rospy.logerr ("vel %.3f %.3f" % (msg.linear.x , msg.angular.z ))

def odom_callback_1(odom):
    global odomx, odomy, odomtheta, odomv, odomw
    odomx = odom.pose.pose.position.x
    odomy = odom.pose.pose.position.y
    odom_quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    odomtheta = tf.transformations.euler_from_quaternion(odom_quaternion)[2]
    odomv = odom.twist.twist.linear.x
    odomw = odom.twist.twist.angular.z
    #rospy.logerr ("odo %.3f %.3f %.3f" % (odomx, odomy, odomtheta))
    #rospy.logerr ("odo %.3f %.3f %.3f %.3f %.3f" % (odomx, odomy, odomtheta, odomv, odomw))
    #rospy.logerr ("odo %.3f %.3f %.3f %.3f    %.3f %.3f %.3f %.3f" % (odomx, odomy, odomtheta, odomv,   v2_odomx, v2_odomy, v2_odomtheta, v2_odomv))



def odom_callback_2(v2_odom):
    global v2_odomx, v2_odomy, v2_odomtheta, v2_odomv, v2_odomw
    v2_odomx = v2_odom.pose.pose.position.x
    v2_odomy = v2_odom.pose.pose.position.y
    v2_odom_quaternion = (v2_odom.pose.pose.orientation.x, v2_odom.pose.pose.orientation.y, v2_odom.pose.pose.orientation.z, v2_odom.pose.pose.orientation.w)
    v2_odomtheta = tf.transformations.euler_from_quaternion( v2_odom_quaternion)[2]
    v2_odomv = v2_odom.twist.twist.linear.x
    v2_odomw = v2_odom.twist.twist.angular.z
    global prev_time, acc_time
    current_time = rospy.Time.now()
    acc_time = acc_time + (current_time - prev_time).to_sec()
    prev_time = rospy.Time.now()

    #rospy.logerr ("odo %.3f %.3f %.3f %.3f    %.3f %.3f %.3f %.3f" % (odomx, odomy, odomtheta, odomv,   v2_odomx, v2_odomy, v2_odomtheta, v2_odomv))
    #rospy.logerr ("odo %.3f %.3f %.3f" % ( v2_odomx, v2_odomy, v2_odomtheta))
    #pose_out.write( "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n" % (acc_time, odomx, odomy, odomtheta, odomv, odomw, v2_odomx, v2_odomy, v2_odomtheta, v2_odomv, v2_odomw ))

    pose_out.write( "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n" % (acc_time, odomx, odomy, odomtheta, odomv, odomw, cmd_vel_v, v2_odomx, v2_odomy, v2_odomtheta, v2_odomv, v2_odomw, v2_cmd_vel_v ))




'''
def pose_callback(pose):
    global posex, posey, posetheta

    global pose_out
    global odomv, odomw
    global v2_odomv, v2_odomw
    global prev_time, acc_time
    current_time = rospy.Time.now()
    acc_time = acc_time + (current_time - prev_time).to_sec()
    prev_time = rospy.Time.now()
    
    posex = pose.pose.position.x
    posey = pose.pose.position.y
    pose_quaternion = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
    posetheta = tf.transformations.euler_from_quaternion(pose_quaternion)[2]
    if ADJUST_FOR_WHILL_PASSENGER == True:
        newX = posex + PASSENGER_DIST * math.cos( posetheta ) # calc new X
        newY = posey + PASSENGER_DIST * math.sin( posetheta ) # calc new Y
        posex=newX
        posey=newY
    pose_out.write( "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n" % (acc_time, posex, posey, posetheta, odomv, odomw, v2_posex, v2_posey, v2_posetheta, v2_odomv, v2_odomw ))

    
def v2_vel_callback(odom):
    global v2_odomx, v2_odomy, v2_odomtheta, v2_odomv, v2_odomw
    #v2_odomx = odom.position.x
    #v2_odomy = odom.position.y
    #v2_odom_quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    #v2_odomtheta = tf.transformations.euler_from_quaternion(v2_odom_quaternion)[2]
    v2_odomv = odom.linear.x
    v2_odomw = odom.angular.z
    #rospy.logerr ("v2_odo %.3f %.3f %.3f %.3f %.3f" % (v2_odomx, v2_odomy, v2_odomtheta, v2_odomv, v2_odomw))

def v2_pose_callback( msg ):
    global v2_posex, v2_posey, v2_posetheta
    
    v2_posex = msg.position.x
    v2_posey = msg.position.y
    v2_pose_quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    v2_posetheta = tf.transformations.euler_from_quaternion(v2_pose_quaternion)[2]

    #rospy.logerr ("v2_pose %.3f %.3f %.3f" % (v2_posex, v2_posey, v2_posetheta))

'''

    
def run():
    
    global out_folder
    global min_dist, min_rot
    global pose_out
    global prev_time
    
    rospy.init_node('extract', anonymous=True)

    odom_topic_1    = rospy.get_param('odom_topic_1', 'v1/odom')
    odom_topic_2    = rospy.get_param('odom_topic_2', 'v2/odom')

    cmd_vel_topic_1    = rospy.get_param('cmd_vel_topic_1', 'v1/cmd_vel')
    #cmd_vel_topic_2    = rospy.get_param('cmd_vel_topic_2', 'v2/cmd_vel')

    cmd_vel_topic_2    = rospy.get_param('cmd_vel_topic_2', '/v1/cmd_vel_mux/path_controller')

    
        
    #pose_topic    = rospy.get_param('pose', '/filtered_ndt_current_pose')
    #v2_odom_topic = rospy.get_param('v2_odom_topic', '/v2/vel')
    #v2_pose_topic = rospy.get_param('v2_pose', '/v2/pose')
    
    out_folder = rospy.get_param('~output_folder', 'output/')
    delete_folder = rospy.get_param('~delete_old_folder', False)

    output_file = rospy.get_param('~output_file', 'pair_log.txt')

    rospy.Subscriber( odom_topic_1, Odometry, odom_callback_1 )
    rospy.Subscriber( odom_topic_2, Odometry, odom_callback_2 )
    
    #rospy.Subscriber(pose_topic, PoseStamped, pose_callback)
    rospy.Subscriber(cmd_vel_topic_1, Twist, cmd_vel_callback_1)
    rospy.Subscriber(cmd_vel_topic_2, Twist, cmd_vel_callback_2)
    #rospy.Subscriber(v2_pose_topic, Pose, v2_pose_callback)

    if os.path.isfile(output_file) == True:
        print("Output file '%s' exists - exiting" % output_file)#(os.path.realpath(out_folder)))
        sys.exit(1)

    prev_time = rospy.Time.now()
    pose_out  = open(output_file, "w")
    rospy.spin()

    pose_out.close()
    print "\nExiting: extract.py\n\n\n"
    
    
  

if __name__ == '__main__':
    print ("Running")
    run()
