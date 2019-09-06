#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

class extract
{
public:
	extract() :
		nh("~")
	{
	  odom_frame_id = "odom";
	  nh.param("odom_frame_id", odom_frame_id, odom_frame_id );
	  sub_odom = nh.subscribe("/odom", 1, &extract::odomCallback, this);

	  sub_joystick     = nh.subscribe("/joystick", 1, &extract::joystickCallback, this);
	  sub_output_power = nh.subscribe("/output_power", 1, &extract::output_powerCallback, this);
	  sub_battery      = nh.subscribe("/battery_charge_state", 1, &extract::batteryCallback, this);
	  sub_speed_up     = nh.subscribe("/speed_up_cnt", 1, &extract::speed_upCallback, this);
	  sub_speed_down   = nh.subscribe("/speed_down_cnt", 1, &extract::speed_downCallback, this);
	  sub_alarm        = nh.subscribe("/alarm", 1, &extract::alarmCallback, this);
	  
	};

	void spinOnce()
	{
	  if(init){
	  ros::Time current_time = ros::Time::now();
	  std::cerr << odom.pose.pose.position.x << " ";
	  std::cerr << odom.pose.pose.position.y << " ";
	  //std::cerr << yaw << " ";
	  std::cerr << odom.twist.twist.linear.x << " ";
	  std::cerr << odom.twist.twist.angular.z << " ";
	  
	  std::cerr << joystick_msg.x          << " ";
	  std::cerr << joystick_msg.y          << " ";
	  std::cerr << output_power_msg.x          << " ";
	  std::cerr << output_power_msg.y          << " ";
	  std::cerr << battery_charge_msg.data << " ";
	  std::cerr << speed_up_cnt_msg.data   << " ";
	  std::cerr << speed_down_cnt_msg.data << " ";
	  std::cerr << alarm_cnt_msg.data      << " ";
	  	  
	  std::cerr << std::endl;
	  }
	};


private:
  ros::NodeHandle nh;
  std::string odom_frame_id;
  nav_msgs::Odometry odom;
  geometry_msgs::Vector3 joystick_msg;
  geometry_msgs::Vector3 output_power_msg;
  std_msgs::Int32 battery_charge_msg;
  std_msgs::Int32 speed_up_cnt_msg; 
  std_msgs::Int32 speed_down_cnt_msg;
  std_msgs::Int32 alarm_cnt_msg;

  ros::Subscriber sub_joystick    ;
  ros::Subscriber   sub_output_power;
  ros::Subscriber   sub_battery     ;
  ros::Subscriber   sub_speed_up    ;
  ros::Subscriber   sub_speed_down  ;
  ros::Subscriber   sub_alarm       ;
  ros::Subscriber sub_odom;
  bool init=false;
  


  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void joystickCallback(const geometry_msgs::Vector3::ConstPtr& msg);
  void output_powerCallback(const geometry_msgs::Vector3::ConstPtr& msg);
  void batteryCallback(const std_msgs::Int32::ConstPtr& msg);
  void speed_upCallback(const std_msgs::Int32::ConstPtr& msg);
  void speed_downCallback(const std_msgs::Int32::ConstPtr& msg);
  void alarmCallback(const std_msgs::Int32::ConstPtr& msg);

};

void extract::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom = *msg;
	init=true;
}

void extract::joystickCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	joystick_msg = *msg;
}

void extract::output_powerCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	output_power_msg = *msg;
}

void extract::batteryCallback(const std_msgs::Int32::ConstPtr& msg)
{
  battery_charge_msg = *msg;
}

void extract::speed_upCallback(const std_msgs::Int32::ConstPtr& msg)  
{
  speed_up_cnt_msg = *msg;  
}

void extract::speed_downCallback(const std_msgs::Int32::ConstPtr& msg)
{
  speed_down_cnt_msg= *msg;
}

void extract::alarmCallback(const std_msgs::Int32::ConstPtr& msg)
{
  alarm_cnt_msg = *msg;
}


int main( int argc, char *argv[] )
{
	ros::init(argc, argv, "extract_odometry");
	extract ext;

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ext.spinOnce();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 1;
}



