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

#ifndef RRC_CONTROLLER_HPP
#define RRC_CONTROLLER_HPP

#include <iostream>
#include <memory>  // unique_ptr, make_unique(), shared_ptr, make_shared()
#include <boost/thread.hpp>
#include <boost/timer/timer.hpp>
#include "serial_port.hpp"
#include "rrc_cmd.hpp"

#include "rrc_system_ros.hpp"
#include "rona_util.hpp"  // librona からの一部移植

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

struct Odometry2d {
    rona::Velocity2d vel;
    rona::Pose2d pose;
    double travel_dist;  // 累積走行距離 [m]
    double travel_ang;   // 累積回転角度 [rad]

};

class RrcController {
public:
    RrcController(const std::string &dev_name, std::size_t print_interval = 1,
                  int send_hz = 100, double cmd_timeout = 0.2, double recv_timeout = 0.5)
        : print_interval_(print_interval), send_hz_(send_hz), cmd_timeout_(cmd_timeout), recv_timeout_(recv_timeout)
    {
        if (print_interval_ == 0)
            print_interval_ = 1;

        auto err_code = port_.open(dev_name);
        if (err_code)
        {
            std::cerr << "ERROR: Cannot open port: " << dev_name
                      << ", error_code: " << err_code.message() << std::endl;
            exit(1);
        }
        std::cout << "Open port: " << dev_name << std::endl;
        initPort();
    }

    ~RrcController()
    {
        port_.close();
    }

    void run();

private:
    void sendLoop();
    void recvLoop();

    void initPort();

    void sendInit();

    void handleEvRes(const std::string &read_line);
    void handleStatusRes(const std::string &read_line);

  void pct2vel(int v_pct, int w_pct, rona::Velocity2d *vel);
  void vel2pct(const rona::Velocity2d &vel, int *v_pct, int *w_pct);
  void enc2odo_vel(int enc_l, int enc_r, int *prev_enc_l, int *prev_enc_r, double step_time, rona::Pose2d *odom, rona::Velocity2d *vel );

    SerialPort port_;
    RrcSystemRos sys_;

    std::unique_ptr<boost::thread> recv_thread_;

    std::size_t recv_cnt_ = 0;
    std::size_t print_interval_;

    RrcStateT rrc_state_ = STATE_POWER_ON_WAIT;
    bool done_init_ = false;

    // パーセント単位変換用
    // TODO: 速度モードごとの最高速度の測定、キャリブレーション
    double rrc_max_v_ = 1.667;  // 1.667 m/s = 6 km/h
    double rrc_max_w_ = rona::deg2rad(45.0);  // TODO: キャリブレーション

    int send_hz_;
    double cmd_timeout_;
    boost::timer::cpu_timer recv_timer_;
    double recv_timeout_;  // タイムアウト時は exit() するので余裕を持った時間を設定

    rona::SimpleOdometer odometer_;
    double odom_step_time_ = 0.01;  // 10 ms
  int prev_enc_r=0;
  int prev_enc_l=0;
  bool encoder_initialized = false;
  double TREAD=0.555 ;
  rona::Velocity2d vel;
  rona::Pose2d odom ={0.0, 0.0, 0.0};

  ros::Publisher battery_charge_state_pub;
  ros::Publisher joystick_pub;
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom_msg;

  bool AUTONOMOUS_MODE = false ;

  RrcRecvStatus recv_status ;

  /*  
  geometry_msgs::Vector3 joystick_msg;
  geometry_msgs::Vector3 output_power_msg;
  std_msgs::Int32 battery_charge_msg;
  std_msgs::Int32 speed_up_cnt_msg; 
  std_msgs::Int32 speed_down_cnt_msg;
  std_msgs::Int32 alarm_cnt_msg;
  */
};

#endif // RRC_CONTROLLER_HPP
