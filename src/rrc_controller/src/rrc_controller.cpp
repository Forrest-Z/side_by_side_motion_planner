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

#include "rrc_controller.hpp"

#include <boost/program_options.hpp>

namespace bo = boost::program_options;

void RrcController::run()
{
  std::cout << "Start receive loop." << std::endl;
  recv_thread_.reset(new boost::thread(&RrcController::recvLoop, this));
  
  std::cout << "Start send loop." << std::endl;
  sendLoop();
  

  recv_thread_->join();  // 受信スレッドの終了待機
}



void RrcController::sendLoop()
{
    RrcSystemRos::Rate rate(send_hz_);

    // TODO: プログラム起動前に EV:PowerOnSuccess している対策として state コマンドを使用する？
    rrc_state_ = STATE_RUNNING;
    while (sys_.ok())
    {
      //std::cout << "\nState " << STATE_RUNNING << " rrc_state " << rrc_state_ << " done_init " << done_init_ << std::endl;
      //sendInit();
        // 初期設定
        if (rrc_state_ == STATE_RUNNING && done_init_ == false)
        {
	  sendInit();
	  std::cout << "\n\n Init . . . \n\n " << std::endl;
	  //exit(0);
        }

        if (rrc_state_ == STATE_RUNNING && done_init_ == true)
        {
            // 目標速度取得
            rona::Velocity2d cmd_vel{0.0, 0.0};
            double cmd_time = 0.0;
            sys_.getCmdVel(&cmd_vel, &cmd_time);
	    
            // タイムアウト時は速度をゼロにする
            if (sys_.timeNow() - cmd_time > cmd_timeout_) //0.2sec
            {
                cmd_vel.v = 0.0;
                cmd_vel.w = 0.0;
            }
	    
            int v_pct, w_pct;
            vel2pct(cmd_vel, &v_pct, &w_pct);
	    
            // TODO: 速度モードの変更に対応
	    //std::cerr << "\nv " << cmd_vel.v << " " << cmd_vel.w << " ";
	    //std::cout << v_pct << " " << w_pct << " ";
            // 速度コマンド送信
	    
	    
	    if ( recv_status.v_pct_in <= 2 && recv_status.v_pct_in >=-2 && recv_status.w_pct_in <= 2 && recv_status.w_pct_in >= -2) {
	      AUTONOMOUS_MODE = true ;
	      port_.write(rrc_cmd::encOutput(v_pct, -w_pct, false));
	      }
	    /*
            bool do_print = (recv_cnt_ % print_interval_ == 0);
            if (do_print)
            {
                std::cout << std::string(80, '=') << "\n";
                std::cout << std::showpos;  // プラスも符号表記
                std::cout << "SEND vel: " << cmd_vel.v << " [m/s], " << rona::rad2deg(cmd_vel.w) << " [deg/s]\n";
                std::cout << "SEND vel: " << v_pct << " [%], " << w_pct << " [%]\n";
                std::cout << std::noshowpos;  // 符号表記を元に戻す
            }
	    */
        }

        rate.sleep();
    }
}


//recvER:output size: 9 line: ER:outputERROR: ER recv: ER:output
//recvSFE00754000000E size: 15 line: SFE00754000000EERROR: Unknown recv: SFE00754000000E

void RrcController::recvLoop()
{
    while (sys_.ok())
    {
        sys_.spinOnce();  // Subscriber のコールバックに必要

        std::string read_line;
        int read_size = port_.readUntilTimeOut(read_line, "\n", 20);  // 20 ms

	//#if 0  // debug
	//        std::cout << "recv: " << read_line << std::endl;
	//O#endif

	    //std::cout << "  \n recv: " << read_line << " size: " << read_size << " line: " <<read_line;

        if (read_size != 0)
        {
            ++recv_cnt_;
            recv_timer_.start();  // 受信タイムアウト判定用

            // EV 応答
            if (read_line.substr(0, 3) == "EV:")
            {
                std::cout << "EV recv: " << read_line << std::endl;
                handleEvRes(read_line);
            }
            // OK 応答
	    else if (read_line.substr(0, 3) == "OK:")
	      {
                //std::cout << "OK recv: " << read_line << std::endl;
            }
	    
            // ER 応答
            else if (read_line.substr(0, 3) == "ER:")
            {
                std::cout << " ERROR: ER recv: " << read_line << std::endl;
            }
            // SA, SB 応答（10 ms 周期）
            // TODO: typeS に対応
            //else if (read_line.substr(0, 3) == "SA:" || read_line.substr(0, 3) == "SB:")
	    else if (read_line.substr(0, 1) == "S")
            {
                 handleStatusRes(read_line);
            }
            // その他
            else
            {
                std::cout << "ERROR: Unknown recv: " << read_line << std::endl;
            }
        }

        // 受信タイムアウト判定によるウォッチドッグ
        if (done_init_ == true)
        {
            double recv_interval = recv_timer_.elapsed().wall / 1e9;  // [s]
            if (recv_interval > recv_timeout_)
            {
                std::cerr << "ERROR: Receive timeout: " << recv_interval << " sec" << std::endl;
                exit(1);
            }
        }

        sys_.sleepFor(1);  // 1 ms
    }
}

void RrcController::initPort()
{
    port_.setBaudRate(115200);
    port_.setFlowControl(SerialPort::FLOW_CTRL_NONE);
    port_.setParity(SerialPort::PARITY_NONE);
    port_.setStopBits(SerialPort::STOP_BITS_ONE);
    port_.setCharacterSize(8);
}

void RrcController::sendInit()
{
  /*
    // ウォッチドッグを有効にする
    port_.write(rrc_cmd::encWatchdog(50, false));  // 50 * 10 = 500 ms
    sys_.sleepFor(10);  // 10 ms

    // 連続通信エラー時にエラーイベント応答を発生させる
    port_.write(rrc_cmd::encRelayError(9, false));
    sys_.sleepFor(10);  // 10 ms

  */

  port_.write(rrc_cmd::s1set(1, false));
  sys_.sleepFor(20);  // 10 ms
  // Get time stamp
  port_.write(rrc_cmd::s1set(1, false));
  sys_.sleepFor(20);  // 10 ms
  std::cout << "\nCommand:  " << rrc_cmd::s1set(1, false);
  
  // Get status of the joystick
  port_.write(rrc_cmd::s2set(1,1,1,1, false));
  sys_.sleepFor(20);  // 10 ms
  std::cout << "\nCommand:  " << rrc_cmd::s2set(1,0,1,1, false);
  
    

  
  //Get status from the encoders
    port_.write(rrc_cmd::s3set(false));
    sys_.sleepFor(20);  // 10 ms
    std::cout << "\n\n command:  " << rrc_cmd::s3set(false); // << std::endl;
    //std::cout << "hi";
    
    
    // Drive with Joystick (Setting joystick ranges)
    //port_.write(rrc_cmd::encLimit(50, 50, 255, 255, 255, false));
    port_.write(rrc_cmd::encLimit(100, 100, 255, 255, 255, false)); 
    sys_.sleepFor(20);  // 10 ms 
    std::cerr << "\nCommand sent:  " << rrc_cmd::encLimit(100, 100, 255, 255, 255, false);

    // Get status feedback from wheelchair 
    port_.write(rrc_cmd::encStatus('S', true, false));
    sys_.sleepFor(20);  // 10 ms
    std::cout << "\nCommand:  " << rrc_cmd::encStatus('S', true, false); // << std::endl;


      

    /*
    // OK応答を省略してチェックサムを有効にする
    port_.write(rrc_cmd::encStableMode(true, false));
    sys_.sleepFor(10);  // 10 ms

  */


//#if 0
    // 速度モードを最低速に設定
//    port_.write(rrc_cmd::encSpeedMode(1, true, true));
//    sys_.sleepFor(10);  // 10 ms
//#else

/*
    // 速度モードを最高速に設定
    port_.write(rrc_cmd::encSpeedMode(5, true, true));
    sys_.sleepFor(10);  // 10 ms
    std::cout << "\n\n SpeedMode:  " << rrc_cmd::encSpeedMode(5, true, true) << std::endl;
*/
    //exit(0);
    //#endif

      done_init_ = true;
}

void RrcController::handleEvRes(const std::string &read_line)
{
    if (read_line == "EV:PowerOnWait" || read_line == "EV:Initialized")
    {
        rrc_state_ = STATE_POWER_ON_WAIT;
        done_init_ = false;
    }
    else if (read_line == "EV:PowerOnTrigger")
    {
        rrc_state_ = STATE_POWER_ON;
    }
    else if (read_line == "EV:PowerOnSuccess")
    {
        rrc_state_ = STATE_RUNNING;
    }
    else if (read_line.substr(0, 15) == "EV:PowerOnError" || read_line == "EV:PowerOnTimeout"
             || read_line.substr(0, 17) == "EV:PowerOffWaitBy")
    {
        rrc_state_ = STATE_POWER_OFF_WAIT;
    }
    else if (read_line == "EV:PowerOffFinished")
    {
        rrc_state_ = STATE_POWER_OFF;
    }
    else
    {
        std::cout << "INFO: Unknown EV response." << std::endl;
    }
}

// 10 ms 周期
void RrcController::handleStatusRes(const std::string &read_line)
{
  if (!rrc_cmd::recvChecksumOk(read_line))
    {
      std::cerr << "ERROR: Status response checksum NG: " << read_line << std::endl;
      return;
    }
    if ((read_line.substr(0, 1) == "S" && read_line.size() != 21) )
    {
        std::cerr << "ERROR: Invalid status response size: " << read_line << std::endl;
        return;
    }
    // TODO: typeS に対応

    //RrcRecvStatus recv_status = rrc_cmd::decStatusAB(read_line);
    recv_status = rrc_cmd::decStatusAB(read_line);

    if (encoder_initialized){    
      enc2odo_vel( recv_status.enc0_cnt, recv_status.enc1_cnt, &prev_enc_l, &prev_enc_r, odom_step_time_, &odom, &vel );      
      bool do_print = (recv_cnt_ % print_interval_ == 0);
      if (do_print)
	{
	  std::cout << std::string(80, '-') << "\n";
	  std::cout << std::showpos;  // プラスも符号表記
	  std::cout << "RECV vel: " << recv_status.v_pct_out << " [%], " << recv_status.w_pct_out << " [%]\n";
	  std::cout << "RECV vel: " << vel.v << " [m/s], " << rona::rad2deg(vel.w) << " [deg/s]\n";
	  std::cout << "odom: " << odom.x << " [m], " << odom.y << " [m], " << rona::rad2deg(odom.th) << " [deg]\n";
	  std::cout << std::noshowpos;  // 符号表記を元に戻す
	}
      
      sys_.pub(odom, vel, &recv_status, sys_.timeNow());

      /*
	std::cerr << recv_status.v_pct_in << " ";
	std::cerr << recv_status.w_pct_in << " ";
	std::cerr << recv_status.v_pct_out << " ";
	std::cerr << recv_status.w_pct_out << " ";
	std::cerr << recv_status.voltage_gauge << " ";
	std::cerr << recv_status.speed_mode << " ";
	std::cerr << recv_status.speed_up_cnt << " ";
	std::cerr << recv_status.speed_down_cnt << " ";
	std::cerr << recv_status.alarm_cnt << " ";
	std::cerr << recv_status.enc0_cnt<< " ";
	std::cerr << recv_status.enc1_cnt << " ";	
	// std::cerr << diff_l * -0.00169 / 0.01 << " ";
	// std::cerr << diff_r * -0.00169 / 0.01 << " ";
	
	std::cerr << "\n";
      */
    }
    else
      encoder_initialized = true;

    prev_enc_l = recv_status.enc0_cnt;
    prev_enc_r = recv_status.enc1_cnt;


    
    if ( (recv_status.v_pct_in >= 2 || recv_status.v_pct_in <=-2 || recv_status.w_pct_in >= 2 || recv_status.w_pct_in <= -2)   && AUTONOMOUS_MODE ){
      port_.write(rrc_cmd::encLimit(100, 100, 255, 255, 255, false)); 
      sys_.sleepFor(10);  // 10 ms 
      //std::cerr << "\nJoystick Mode:  " << rrc_cmd::encLimit(100, 100, 255, 255, 255, false);
      AUTONOMOUS_MODE = false ;
    }
    

    
}



void RrcController::enc2odo_vel(int enc_l, int enc_r, int *prev_enc_l, int *prev_enc_r, double step_time, rona::Pose2d *odom, rona::Velocity2d *vel )
{
  int diff_r = enc_r - *prev_enc_r;
  if( diff_r > 200 )
    diff_r -= 256;
  if( diff_r < -200 )
    diff_r += 256;
  
  int diff_l = enc_l - *prev_enc_l;
  if( diff_l > 200 )
    diff_l -= 256;
  if( diff_l < -200 )
    diff_l += 256;

  double vel_l = diff_l * -0.00169 / 0.01 ;
  double vel_r = diff_r * -0.00169 / 0.01 ;

  vel->v = (vel_l + vel_r) / 2.0; 
  vel->w = (vel_r - vel_l) / TREAD; 
  
  odom->th += vel->w * step_time;
  odom->th = rona::wrapToNegPosPi(odom->th);
  odom->x += vel->v * cos(odom->th) * step_time;
  odom->y += vel->v * sin(odom->th) * step_time;

}



void RrcController::pct2vel(int v_pct, int w_pct, rona::Velocity2d *vel)
{
    vel->v = v_pct * rrc_max_v_ / 100.0;;
    vel->w = -1.0 * w_pct * rrc_max_w_ / 100.0;  // 符号反転
}

//ToDO Implement curves for velocites according to velocity level
void RrcController::vel2pct(const rona::Velocity2d &vel, int *v_pct, int *w_pct)
{
  double a_vl3 =-34.59;
  double b_vl3 = 123.61;
  double c_vl3 = 1.57 ;
  double v_a=a_vl3;
  double v_b=b_vl3;
  double v_c=c_vl3;

  *v_pct = vel.v*vel.v*v_a + vel.v*v_b + v_c ; 

  double a_wl3 =-2.13;
  double b_wl3 = 83.75;
  double c_wl3 = 0.51 ;
  double w_a=a_wl3;
  double w_b=b_wl3;
  double w_c=c_wl3;
  
  *w_pct = vel.w*vel.w*w_a + vel.w*w_b + w_c ; 

  //*v_pct = vel.v / rrc_max_v_ * 100.0;
  //*w_pct = -1.0 * vel.w / rrc_max_w_ * 100.0;  // 符号反転
}


int main(int argc, char **argv)
{
    // 小数点表記に設定
    std::cout << std::fixed;
    std::cerr << std::fixed;



#if 0

    // 引数が正しくない場合は使い方を表示
    if (argc != 2)
    {
        std::cout << "USAGE:" << "\n";
//        std::cout << "  $ " << argv[0] << " <yaml_file>" << "\n";
//        std::cout << "  e.g.)  $ " << argv[0] << " params.yaml" << "\n";
        std::cout << "  $ " << argv[0] << " <dev_file>" << "\n";
        std::cout << "  e.g.)  $ " << argv[0] << " /dev/ttyACM0" << std::endl;
        return 1;
    }
    std::string dev_name(argv[1]);
    ProtoVerT proto = PROTO_B2;
#else
    //std::cout << "  \n\n Hey " << "\n\n";
    bo::options_description options("options");
    options.add_options()
            ("help,h", "show help.")
            ("device,d", bo::value<std::string>()->default_value("/dev/ttyACM0"), "device file path.")
            ("output,o", bo::value<std::size_t>()->default_value(100), "output print interval. 0/1: all, 2: half (1/2), ...");
    bo::variables_map vmap;
    try
    {
        bo::store(bo::parse_command_line(argc, argv, options), vmap);  // コマンドライン引数を解析、格納の定義
        bo::notify(vmap);  // 格納
    }
    catch (std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    if(vmap.count("help"))
    {
        std::cout << options << std::endl;
        return 0;
    }

    std::string dev_name = vmap["device"].as<std::string>();

    std::size_t print_interval = vmap["output"].as<std::size_t>();
    if (print_interval == 0)
        print_interval = 1;
#endif

    ros::init(argc, argv, "rrc_controller");

    RrcController controller(dev_name, print_interval);
    controller.run();

    return 0;
}

