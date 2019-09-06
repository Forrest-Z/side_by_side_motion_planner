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

#ifndef RRC_SYSTEM_ROS_HPP
#define RRC_SYSTEM_ROS_HPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "rrc_cmd.hpp"  // 
#include "rona_util.hpp"  // librona からの一部移植

// ミドルウェア抽象化
class RrcSystemRos {
public:
    RrcSystemRos()
    {
        sub_cmd_ = nh_.subscribe("cmd_vel", 1, &RrcSystemRos::cbCmdVel, this);
        pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);

	joystick_pub = nh_.advertise<geometry_msgs::Vector3>("joystick", 1);
	output_power_pub = nh_.advertise<geometry_msgs::Vector3>("output_power", 1);
	battery_charge_state_pub = nh_.advertise<std_msgs::Int32>("battery_charge_state", 1);
	speed_up_pub = nh_.advertise<std_msgs::Int32>("speed_up_cnt", 1);
	speed_down_pub = nh_.advertise<std_msgs::Int32>("speed_down_cnt", 1);
	alarm_pub = nh_.advertise<std_msgs::Int32>("alarm", 1);
	
    }

    ~RrcSystemRos() = default;

    bool ok()
    {
        return ros::ok();
    }

    void spinOnce()
    {
        return ros::spinOnce();
    }

    using Rate = ros::Rate;

    void sleepFor(int time_ms)
    {
        ros::Duration(time_ms / 1000.0).sleep();
    }

    double timeNow()
    {
        return ros::Time::now().toSec();
    }

    void getCmdVel(rona::Velocity2d *vel, double *time);
  void pub(const rona::Pose2d &pose, const rona::Velocity2d &vel, RrcRecvStatus *recv_status, double time);

private:
    void cbCmdVel(const geometry_msgs::Twist::ConstPtr &cmd_vel);

    rona::Velocity2d cmd_vel_;  // 目標速度
    double cmd_stamp_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_cmd_;
    ros::Publisher pub_odom_;
    ros::Publisher joystick_pub;

    ros::Publisher battery_charge_state_pub; 
  ros::Publisher output_power_pub ; 
  ros::Publisher speed_up_pub ;
  ros::Publisher speed_down_pub ;
  ros::Publisher alarm_pub ;


    tf::TransformBroadcaster tf_broadcaster_;

    std::string base_frame_ = "base_link";
    std::string odom_frame_ = "odom";

  geometry_msgs::Vector3 joystick_msg;
  geometry_msgs::Vector3 output_power_msg;
  std_msgs::Int32 battery_charge_msg;
  std_msgs::Int32 speed_up_cnt_msg; 
  std_msgs::Int32 speed_down_cnt_msg;
  std_msgs::Int32 alarm_cnt_msg;

};

#endif // RRC_SYSTEM_ROS_HPP
