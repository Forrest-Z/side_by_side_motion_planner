#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace YP
{
#include <ypspur.h>
}

double get_time( void )
{
	struct timeval current;
	gettimeofday( &current, NULL );
	return current.tv_sec + current.tv_usec / 1000000.0;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO( "new speed: [%0.2f][%0.2f]", msg->linear.x, msg->angular.z);
	YP::YPSpur_vel( msg->linear.x, msg->angular.z );
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RosYPSpur");
	ros::NodeHandle n("~");
	
	double vel = 0.2, angvel = 0.4, acc = 0.4, angacc = 0.8;
	int hz = 200;
	std::string port = "/dev/ttyACM0";
	std::string ypspur = "/usr/local/bin/ypspur-coordinator";
	std::string odom_id = "odom";
	std::string base_link_id = "base_link";
	std::string origin_id = "";
	double ad_gain[8];
	double ad_offset[8];
	bool ad_enable[8];

	n.param( "vel", vel, vel );
	n.param( "acc", acc, acc );
	n.param( "angvel", angvel, angvel );
	n.param( "angacc", angacc, angacc );
	n.param( "port", port, port );
	n.param( "ypspur", ypspur, ypspur );
	n.param( "hz", hz, hz );
	n.param( "odom_id", odom_id, odom_id );
	n.param( "base_link_id", base_link_id, base_link_id );
	n.param( "origin_id", origin_id, origin_id );
	std::string ad_mask("");
	for(int i = 0; i < 8; i ++)
	{
		n.param( std::string("ad_gain") + std::to_string(i), ad_gain[i], 1.0);
		n.param( std::string("ad_offset") + std::to_string(i), ad_offset[i], 0.0);
		n.param( std::string("ad_enable") + std::to_string(i), ad_enable[i], false);
		if(ad_enable[i])
			ad_mask = std::string("1") + ad_mask;
		else
			ad_mask = std::string("0") + ad_mask;
	}

	if(YP::YPSpur_init() < 0)
	{
		pid_t pid = fork();
		if( pid == 0 )
		{
			execl( ypspur.c_str(), ypspur.c_str(), 
				"-d", port.c_str(), "--high-resolution", "--admask", ad_mask.c_str(), NULL );
			ROS_ERROR( "failed to run ypspur-coordinator\nYP-Spur is available on https://openspur.org/redmine/projects/ypspur/" );
			return 1;
		}
		sleep( 2 );
		if(YP::YPSpur_init() < 0)
		{
			ROS_ERROR( "failed to start ypspur-coordinator" );
			return 1;
		}
	}
	ROS_INFO( "ypspur-coordinator conneceted" );

	ROS_INFO( "max vel: %0.2f, acc: %0.2f, angvel: %0.2f, angacc: %0.2f", 
		vel, acc, angvel, angacc);

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_id, 1);
	ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
	ros::Publisher ad_pub = n.advertise<std_msgs::Float32MultiArray>("ad", 1);
	ros::Publisher wt_pub = n.advertise<sensor_msgs::JointState>("wheel_joint", 1);

	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmdVelCallback);

	tf::TransformListener tf_listener;

	// Create the odometry transform data to send to tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = odom_id;
	odom_trans.child_frame_id = base_link_id;

	// Create the odometry message on odom
	nav_msgs::Odometry odom;
	odom.header.frame_id = odom_id;
	odom.child_frame_id = base_link_id;

	YP::YPSpur_set_vel( vel );
	YP::YPSpur_set_accel( acc );
	YP::YPSpur_set_angvel( angvel );
	YP::YPSpur_set_angaccel( angacc );

	ros::Rate loop_rate(hz);

	double timeDiff;
	timeDiff = ros::Time::now().toSec() - get_time();

	while (ros::ok())
	{
		double x, y, th, v, w, t;
		ros::Time current_time;
		
		// Get odometry from YP-Spur
		t = YP::YPSpur_get_pos(YP::CS_BS, &x, &y, &th);
		YP::YPSpur_get_vel(&v, &w);
		if( YP::YP_get_error_state() )
		{
			std::cerr << "ypspur-coordinator stopped\n";
			break;
		}

		current_time.fromSec( t + timeDiff );
		
		// Get torque and force
		geometry_msgs::WrenchStamped wr;
		wr.header.stamp = current_time;
		YP::YPSpur_get_force(&wr.wrench.force.x, &wr.wrench.torque.z);
		wr.wrench.force.y = 0;
		wr.wrench.force.z = 0;
		wr.wrench.torque.x = 0;
		wr.wrench.torque.y = 0;
		wrench_pub.publish(wr);
	
		std_msgs::Float32MultiArray ad;
		ad.layout.dim.resize(1);
		ad.layout.dim[0].label = port;
		ad.layout.dim[0].stride = 1;
		ad.layout.data_offset = 0;
		for(int i = 0; i < 8; i ++)
		{
			if(ad_enable[i])
			{
				float val;
				val = YP::YP_get_ad_value(i) * ad_gain[i] + ad_offset[i];
				ad.data.push_back(val);
			}
		}
		ad.layout.dim[0].size = ad.data.size();
		ad_pub.publish(ad);
		
		//publish data for each wheel in a JointState msg
		sensor_msgs::JointState joint;
		joint.header.frame_id = std::string("");
		joint.name.resize(2);
		joint.velocity.resize(2);
		joint.position.resize(2);
		joint.effort.resize(2);
		joint.name[0] = std::string("wheel_right");
		joint.name[1] = std::string("wheel_left");

		t = YP::YP_get_wheel_ang(&joint.position[0], &joint.position[1]);
		if(t == 0.0) t = current_time.toSec();
		YP::YP_get_wheel_vel(&joint.velocity[0], &joint.velocity[1]);
		YP::YP_get_wheel_torque(&joint.effort[0], &joint.effort[1]);
		joint.header.stamp = ros::Time(t);
		wt_pub.publish(joint);

		// Create a quaternion from theta
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		// Update the odometry transform data to send to tf
		odom_trans.header.stamp = current_time;
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0;
		odom_trans.transform.rotation = odom_quat;

		odom_broadcaster.sendTransform(odom_trans);

		// Update the odometry message on odom
		odom.header.stamp = current_time;
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0;
		odom.pose.pose.orientation = odom_quat;

		odom.twist.twist.linear.x = v;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = w;

		// Publish to odom
		odom_pub.publish(odom);

		// Listern localization result and apply to ypspur
		if( origin_id.length() > 0 )
		{
			tf::StampedTransform transform;
			try{
				tf_listener.waitForTransform(origin_id, base_link_id, current_time, ros::Duration(0.5));
				tf_listener.lookupTransform(origin_id, base_link_id, current_time, transform);

				tfScalar yaw, pitch, roll;
				transform.getBasis().getEulerYPR( yaw, pitch, roll );
				YP::YPSpur_adjust_pos( YP::CS_GL, transform.getOrigin().x(),
						transform.getOrigin().y(),
						yaw );
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

