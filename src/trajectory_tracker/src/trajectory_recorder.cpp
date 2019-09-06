/*
 * receiving rear wheels velocity from can data and current vehicle pose and record trajectory data.
 * written by Naoki Akai
 */

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#define TREAD 1.3

//double x, y, z, vl, vr, yr, odom_v, odom_w;
double x,y,z;
//bool use_can = false;
FILE *fp;
ros::Time start;
/*
void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr &vxrl_msg, const geometry_msgs::Vector3Stamped::ConstPtr &vxrr_msg)
{
	double vl, vr;

	vl = vxrl_msg->vector.x * 1000.0 / 3600.0;
	vr = vxrr_msg->vector.x * 1000.0 / 3600.0;
	v = (vr + vl) / 2.0;
	w = (vr - vl) / TREAD;
}
 */

//Callback function for topic /can_data/vxrl
/*
void vl_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
	vl = msg->vector.x * 1000.0 / 3600.0;
}

//Callback function for topic /can_data/vxrr
void vr_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
	vr = msg->vector.x * 1000.0 / 3600.0;
}

//Callback function forr topci /can_data/yr
void yr_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
	yr = msg->vector.x * M_PI / 180.0;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	odom_v = msg->twist.twist.linear.x;
	odom_w = msg->twist.twist.angular.z;
}
*/ 
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	x = msg->pose.position.x;
	y = msg->pose.position.y;
	z = msg->pose.position.z;
}

void open_file(char *name)
{
	fp = fopen(name, "w");
	if (fp == NULL)
	{
		ROS_ERROR("cannot open file (%s)", name);
		exit(1);
	}
}

/*void write_data(void)
{
	double v, w;

	if (use_can)
	{
		v = (vr + vl) / 2.0;
		w = (vr - vl) / TREAD;
	}
	else
	{
		v = odom_v;
		w = odom_w;
	}
	fprintf(fp, "%lf %lf %lf %lf %lf\n", x, y, z, v, w);
	fprintf(stderr, "%lf %lf %lf %lf %lf\n", x, y, z, v, w);
}*/

void write_data(void)
{
  double secs=(ros::Time::now()-start).toSec();
  fprintf(fp, "%lf %lf %lf %lf\n",secs, x, y, z);
}

void close_file(void)
{
	fclose(fp);
}

int main(int argc, char **argv)
{
  
	if (argv[1] == NULL)
	{
		open_file((char *)"/home/user/trajectory.txt");
	}
	else
	{
		open_file(argv[1]);
	}

	ros::init(argc, argv, "trajectory_recorder");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	start=ros::Time::now();
//	nh.getParam("trajectory_recorder/use_can", use_can);

/*
	message_filters::Subscriber<geometry_msgs::Vector3Stamped> vxrl_sub(nh, "can_data/vxrl", 1000);
	message_filters::Subscriber<geometry_msgs::Vector3Stamped> vxrr_sub(nh, "can_data/vxrr", 1000);
	message_filters::TimeSynchronizer<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> sync(vxrl_sub, vxrr_sub, 1);
	sync.registerCallback(boost::bind(&velocity_callback, _1, _2));
 */

	//ros::Subscriber vl_sub = nh.subscribe("/can_data/vxrl", 1, vl_callback);
	//ros::Subscriber vr_sub = nh.subscribe("/can_data/vxrr", 1, vr_callback);
	//ros::Subscriber yr_sub = nh.subscribe("/can_data/yr", 1, yr_callback);
	//ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odom_callback);
	ros::Subscriber pose_sub = nh.subscribe("/filtered_ndt_current_pose", 1, pose_callback);

	while (ros::ok())
	{
		ros::spinOnce();
		write_data();
		loop_rate.sleep();
	}

	close_file();

	return 0;
}
