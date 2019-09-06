/*********************************************************************
 * Copyright (C) 2015- Future Robotics Technology Center (fuRo),
 *                     Chiba Institute of Technology.
 *                     All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * fuRo ("Confidential Information"). You shall not disclose such
 * Confidential Information and shall use it only in accordance with
 * the terms of the license agreement you entered into with fuRo.
 *
 * @author Yoshitaka Hara
 *********************************************************************/

#include "rrc_system_ros.hpp"

void RrcSystemRos::cbCmdVel(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    cmd_vel_.v = cmd_vel->linear.x;
    cmd_vel_.w = cmd_vel->angular.z;
    cmd_stamp_ = ros::Time::now().toSec();  // Twist はヘッダなし
}

void RrcSystemRos::getCmdVel(rona::Velocity2d *vel, double *time)
{
    *vel = cmd_vel_;
    *time = cmd_stamp_;
}

void RrcSystemRos::pub(const rona::Pose2d &pose, const rona::Velocity2d &vel, RrcRecvStatus *recv_status, double time)
{
    geometry_msgs::Quaternion quat_msg = tf::createQuaternionMsgFromYaw(pose.th);

    // tf
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp.fromSec(time);
    odom_tf.header.frame_id = odom_frame_;
    odom_tf.child_frame_id = base_frame_;
    odom_tf.transform.translation.x = pose.x;
    odom_tf.transform.translation.y = pose.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = quat_msg;


    //tf_broadcaster_.sendTransform(odom_tf);

    // nav_msgs
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp.fromSec(time);
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = pose.x;
    odom_msg.pose.pose.position.y = pose.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = quat_msg;
    odom_msg.twist.twist.linear.x = vel.v;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = vel.w;

    pub_odom_.publish(odom_msg);

    joystick_msg.x          = recv_status->v_pct_in ;
    joystick_msg.y          = recv_status->w_pct_in ;
    output_power_msg.x      = recv_status->v_pct_out ;
    output_power_msg.y      = recv_status->w_pct_out ;
    
    battery_charge_msg.data = (recv_status->voltage_gauge) ;
    speed_up_cnt_msg.data   = recv_status->speed_up_cnt;
    speed_down_cnt_msg.data = recv_status->speed_down_cnt;
    alarm_cnt_msg.data      = recv_status->alarm_cnt;

    joystick_pub.publish(joystick_msg);
    output_power_pub.publish(output_power_msg);
    battery_charge_state_pub.publish(battery_charge_msg);
    speed_up_pub.publish(speed_up_cnt_msg);
    speed_down_pub.publish(speed_down_cnt_msg);
    alarm_pub.publish(alarm_cnt_msg);

}
