#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <safety_limiter/Status.h>
#include <dynamic_object_msgs/DynamicObjectArray.h>


class limiter
{
public:
	limiter();
	~limiter();
	void spin();
private:
	std::string sCloud;
	std::string sOdom;
	std::string sCmdVel;
	std::string sCmdVelOut;
	std::string sDisable;
	std::string sDynamic;
	std::string base_link_id;

	double lin_acc;
	double ang_acc;
	double lin_margin;
	double ang_margin;
	double lin_esc_margin;
	double ang_esc_margin;
	double dynamic_obj_radius_offset;

	int watchdog_cloud;
	int watchdog_odom;
	int watchdog_vel;
	int watchdog_init;

	ros::NodeHandle nh;
	ros::Subscriber sub_vel;
	ros::Subscriber sub_odom;
	ros::Subscriber sub_cloud;
	ros::Subscriber sub_disable;
	ros::Subscriber sub_dynamic;
	ros::Publisher pub_vel;
	ros::Publisher pub_footprint;
	ros::Publisher pub_obstacles;
	ros::Publisher pub_status;
	tf::TransformListener tf;

	nav_msgs::Odometry odom;
	sensor_msgs::PointCloud cloud;
	geometry_msgs::PolygonStamped footprint;
	geometry_msgs::Twist cmd;
	dynamic_object_msgs::DynamicObjectArray dynamic;

	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
	void disableCallback(const std_msgs::Bool::ConstPtr& msg);
	void dynamicCallback(const dynamic_object_msgs::DynamicObjectArray::ConstPtr& msg);
	void check();

	bool disabled;
	bool use_odom;
};

bool XmlRpc_isNumber(XmlRpc::XmlRpcValue &value)
{
	return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
		value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
}

bool isInPolygon(geometry_msgs::Polygon &polygon, geometry_msgs::Point32 &point)
{
	int cn = 0;
	for(int i = 0; i < (int)polygon.points.size() - 1; i ++)
	{
		if((polygon.points[i].y <= point.y && polygon.points[i+1].y > point.y) ||
		    (polygon.points[i].y > point.y && polygon.points[i+1].y <= point.y))
		{
			float lx;
			lx = polygon.points[i].x + 
				(polygon.points[i+1].x - polygon.points[i].x) *
				(point.y - polygon.points[i].y) / (polygon.points[i+1].y - polygon.points[i].y);
			if(point.x < lx) cn ++;
		}
	}
	return ((cn & 1) == 1);
}
geometry_msgs::Point32 sub2d(geometry_msgs::Point32 &a, geometry_msgs::Point32 &b)
{
	geometry_msgs::Point32 ret;
	ret.x = a.x - b.x;
	ret.y = a.y - b.y;
	return ret;
}
template<typename T1 = geometry_msgs::Point32, typename T2 = geometry_msgs::Point32>
float dist2d(T1 &a, T2 &b)
{
	return sqrtf(powf(a.x-b.x,2) + powf(a.y-b.y,2));
}
float dot2d(geometry_msgs::Point32 &a, geometry_msgs::Point32 &b) 
{
	return a.x*b.x + a.y*b.y;
}
float dot2d(geometry_msgs::Point32 a, geometry_msgs::Point32 b) 
{
	return a.x*b.x + a.y*b.y;
}
float cross2d(geometry_msgs::Point32 &a, geometry_msgs::Point32 &b) 
{
	return a.x*b.y - a.y*b.x;
}
float cross2d(geometry_msgs::Point32 a, geometry_msgs::Point32 b) 
{
	return a.x*b.y - a.y*b.x;
}
float dist2d_line(geometry_msgs::Point32 &a, geometry_msgs::Point32 &b, geometry_msgs::Point32 &c)
{
	return (cross2d(sub2d(b, a), sub2d(c, a)) / dist2d(b, a));
}
float dist2d_linestrip(geometry_msgs::Point32 &a, geometry_msgs::Point32 &b, geometry_msgs::Point32 &c)
{
	if(dot2d(sub2d(b, a), sub2d(c, a) ) <= 0) return dist2d(c, a);
	if(dot2d(sub2d(a, b), sub2d(c, b) ) <= 0) return -dist2d(c, b) - 0.005;
	return fabs( dist2d_line(a, b, c) );
}

void movePolygon(geometry_msgs::Polygon &polygon, nav_msgs::Odometry &pos)
{
	float psin = sinf(pos.pose.pose.orientation.z);
	float pcos = cosf(pos.pose.pose.orientation.z);
	for(int i = 0; i < (int)polygon.points.size(); i ++)
	{
		geometry_msgs::Point32 point;
		point = polygon.points[i];

		polygon.points[i].x = pcos * point.x - psin * point.y + pos.pose.pose.position.x;
		polygon.points[i].y = psin * point.x + pcos * point.y + pos.pose.pose.position.y;
	}
}

limiter::limiter():
	nh("~"),
	disabled(false)
{
	nh.param("cloud", sCloud, std::string("/cloud"));
	nh.param("odom", sOdom, std::string("/odom"));
	nh.param("dynamic", sDynamic, std::string(""));
	nh.param("cmd_vel", sCmdVel, std::string("cmd_vel"));
	nh.param("cmd_vel_out", sCmdVelOut, std::string("/cmd_vel"));
	nh.param("disable_command", sDisable, std::string("disable_command"));
	nh.param("base_link_id", base_link_id, std::string("base_link"));
	nh.param("lin_acc", lin_acc, (0.1));
	nh.param("ang_acc", ang_acc, (0.1));
	nh.param("lin_margin", lin_margin, (0.1));
	nh.param("ang_margin", ang_margin, (0.05));
	nh.param("lin_esc_margin", lin_esc_margin, (0.15));
	nh.param("ang_esc_margin", ang_esc_margin, (0.08));
	nh.param("use_odom", use_odom, false);
	nh.param("dynamic_obj_radius_offset", dynamic_obj_radius_offset, 0.5);

	sub_vel = nh.subscribe(sCmdVel, 2, &limiter::cmdVelCallback, this);
	sub_odom = nh.subscribe(sOdom, 2, &limiter::odomCallback, this);
	sub_cloud = nh.subscribe(sCloud, 2, &limiter::cloudCallback, this);
	sub_disable = nh.subscribe(sDisable, 2, &limiter::disableCallback, this);
	if(sDynamic.size() > 0) sub_dynamic = nh.subscribe(sDynamic, 1, &limiter::dynamicCallback, this);
	pub_vel = nh.advertise<geometry_msgs::Twist>(sCmdVelOut, 1);
	pub_footprint = nh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);
	pub_obstacles = nh.advertise<sensor_msgs::PointCloud>("obstacles", 1);
	pub_status = nh.advertise<safety_limiter::Status>("status", 1);

	XmlRpc::XmlRpcValue footprint_xml;
	nh.getParam("footprint", footprint_xml);
	if(footprint_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xml.size() < 3)
	{
		ROS_FATAL("Illigal footprint");
		throw std::runtime_error("Illigal footprint");
	}
	footprint.polygon.points.clear();
	footprint.header.frame_id = base_link_id;
	for(int i = 0; i < (int)footprint_xml.size(); i ++)
	{
		if(!XmlRpc_isNumber(footprint_xml[i][0]) || 
				!XmlRpc_isNumber(footprint_xml[i][1]))
		{
			ROS_FATAL("Illigal footprint value");
			throw std::runtime_error("Illigal footprint value");
		}

		geometry_msgs::Point32 point;
		point.x = (double)footprint_xml[i][0];
		point.y = (double)footprint_xml[i][1];
		point.z = 0;
		footprint.polygon.points.push_back(point);
	}
	footprint.polygon.points.push_back(footprint.polygon.points[0]);
}

limiter::~limiter()
{
}

void limiter::cloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	if(msg->points.size() == 0) return;
	cloud = *msg;

	watchdog_cloud = watchdog_init;
}

void limiter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom = *msg;

	watchdog_odom = watchdog_init;
}

void limiter::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	cmd = *msg;
	watchdog_vel = watchdog_init;
}

void limiter::disableCallback(const std_msgs::Bool::ConstPtr& msg)
{
	disabled = msg->data == true;
}

void limiter::dynamicCallback(const dynamic_object_msgs::DynamicObjectArray::ConstPtr& msg)
{
	dynamic = *msg;
}

void limiter::check()
{
	sensor_msgs::PointCloud _cloud;
	dynamic_object_msgs::DynamicObjectArray _dynamic;
	geometry_msgs::Twist cmd_lim;

	double cmd_v;
	double cmd_w;
	if(use_odom)
	{
		cmd_v = odom.twist.twist.linear.x;
		cmd_w = odom.twist.twist.angular.z;
	}
	else
	{
		cmd_v = cmd.linear.x;
		cmd_w = cmd.angular.z;
	}

	double v = cmd_v;
	double w = cmd_w;
	
	float lin_step = 0.05;
	float ang_step = 0.025;

	double max_dist = 0.5 * v * v / lin_acc + lin_step * 2 + lin_margin;
	double max_rot  = 0.5 * w * w / ang_acc + ang_step * 2 + ang_margin;
	if(max_dist < 0.5) max_dist = 0.5;
	if(max_rot < 0.5) max_rot = 0.5;

	int steps = std::max(fabs(max_dist) / lin_step, fabs(max_rot) / ang_step);
	double time_step = lin_step / (fabs(v) + 0.001);
	if(time_step > ang_step / (fabs(w) + 0.001))
	{
		time_step = ang_step / (fabs(w) + 0.001);
	}

	_dynamic = dynamic;
	try
	{
		tf.waitForTransform(base_link_id, cloud.header.frame_id, ros::Time(0), ros::Duration(0.05));
		tf.transformPointCloud(base_link_id, cloud, _cloud);
		if(sDynamic.size() > 0)
		{
			for(int i = 0; i < (int)dynamic.object.size(); i ++)
			{
				for(int j = 0; j < (int)dynamic.object[i].pose.size(); j ++)
				{
					geometry_msgs::PoseStamped in, out;
					in.pose	= dynamic.object[i].pose[j].pose.pose;
					in.header.stamp = ros::Time(0);
					in.header.frame_id = dynamic.header.frame_id;
					tf.transformPose(base_link_id, in, out);
					_dynamic.object[i].pose[j].pose.pose = out.pose;
				}
				for(auto j = _cloud.points.begin(); j != _cloud.points.end();)
				{
					if(dist2d(*j, _dynamic.object[i].pose[0].pose.pose.position) < dynamic_obj_radius_offset + dynamic.object[i].radius)
					{
						j = _cloud.points.erase(j);
						continue;
					}
					j ++;
				}
			}
		}
	}
	catch (tf::TransformException &e)
	{
		ROS_WARN("TF exception: %s", e.what());
		return;
	}

	// Check collisions and limit velocity
	bool col = false;
	nav_msgs::Odometry pos;
	int step;
	int col_step = -1;

	sensor_msgs::PointCloud obstacle;
	obstacle.header = _cloud.header;
	for(step = 0; step <= steps; step ++)
	{
		bool _col = false;
		ros::Time t = _cloud.header.stamp + ros::Duration(time_step * step);
		// transform footprint
		geometry_msgs::PolygonStamped fp = footprint;
		movePolygon(fp.polygon, pos);

		// intersection check
		for(int j = 0; j < (int)_cloud.points.size(); j ++)
		{
			if(isInPolygon(fp.polygon, _cloud.points[j]))
			{
				if(col_step < 0) col_step = step;
				_col = col = true;
				obstacle.points.push_back(_cloud.points[j]);
				break;
			}
		}
		// intersection check against dynamic objects
		if(sDynamic.size() > 0)
		{
			for(int i = 0; i < (int)_dynamic.object.size(); i ++)
			{
				// linear interpolation
				geometry_msgs::Point32 pos;
				float radius;
				float var;
				bool found = false;
				for(int j = 0; j < (int)_dynamic.object[i].pose.size() - 1; j ++)
				{
					geometry_msgs::PoseWithCovarianceStamped pre = _dynamic.object[i].pose[j];
					geometry_msgs::PoseWithCovarianceStamped post = _dynamic.object[i].pose[j+1];
					if(pre.header.stamp <= t && t < post.header.stamp)
					{
						double r = (t - pre.header.stamp).toSec() /
							(post.header.stamp - pre.header.stamp).toSec();
						pos.x = pre.pose.pose.position.x * (1.0 - r) + post.pose.pose.position.x * r;
						pos.y = pre.pose.pose.position.y * (1.0 - r) + post.pose.pose.position.y * r;
						var = (pre.pose.covariance[0] + pre.pose.covariance[6+1]) * (1.0 - r) + 
							(post.pose.covariance[0] + post.pose.covariance[6+1]) * r;
						radius = _dynamic.object[i].radius * (1.0 - r) + _dynamic.object[i].radius * r;
						found = true;
						break;
					}
				}
				if(found)
				{
					if(isInPolygon(fp.polygon, pos))
					{
						if(col_step < 0) col_step = step;
						_col = col = true;
						obstacle.points.push_back(pos);
						break;
					}
					float d = radius + sqrtf(var) / 2.0;
					geometry_msgs::Point32 l1 = fp.polygon.points[(int)fp.polygon.points.size() - 1];
					for(int j = 0; j < (int)fp.polygon.points.size(); j ++)
					{
						geometry_msgs::Point32 l2 = fp.polygon.points[j];
						if(fabs(dist2d_linestrip(l1, l2, pos)) < d)
						{
							if(col_step < 0) col_step = step;
							_col = col = true;
							obstacle.points.push_back(pos);
							break;
						}
						l1 = l2;
					}
					if(_col) break;
				}
			}
		}
		if(col && col_step > 0) break;
		if(!_col && col_step == 0 )
		{
			if(fabs(v) * (step + 1) * time_step < lin_esc_margin && 
				fabs(w) * (step + 1) * time_step < ang_esc_margin)
			{
				col = false;
				break;
			}
		}
		pos.pose.pose.orientation.z += w * time_step;
		pos.pose.pose.position.x += cos(pos.pose.pose.orientation.z) * v * time_step;
		pos.pose.pose.position.y += sin(pos.pose.pose.orientation.z) * v * time_step;
	}
	cmd_lim = cmd;
	bool limited = false;
	if(col)
	{
		float col_t = col_step * time_step;
		float t_margin_dist = (lin_margin + lin_step) / (fabs(v) + 1e-6);
		float t_margin_ang  = (ang_margin + ang_step) / (fabs(w) + 1e-6);
		
		col_t -= std::min(t_margin_dist, t_margin_ang);
		if(col_t < 0)
		{
			cmd_lim.linear.x = 0;
			cmd_lim.angular.z = 0;
			limited = true;
		}
		else
		{
			float t_stop_dist = fabs(v) / lin_acc;
			float t_stop_ang  = fabs(w) / ang_acc;
			float t_stop = std::max(t_stop_dist, t_stop_ang);
			if(col_t < t_stop)
			{
				float rate = col_t / t_stop;
				cmd_lim.linear.x = v * rate;
				cmd_lim.angular.z = w * rate;
				limited = true;
			}
		}
	}
	safety_limiter::Status status;
	status.header = obstacle.header;
	if( limited && ( fabs(cmd_v) > 0 && fabs(cmd_w) > 0 ) )
	{
		ROS_INFO("Limited (%3.1f %3.1f)->(%3.1f %3.1f)", cmd_v, cmd_w, cmd_lim.linear.x, cmd_lim.angular.z);
		status.status = safety_limiter::Status::DECELERATING;
	}
	else
	{
		obstacle.points.clear();
		status.status = safety_limiter::Status::NORMAL;
	}
	pub_obstacles.publish(obstacle);
	pub_status.publish(status);

	if(watchdog_cloud > 0 && watchdog_odom > 0)
	{
		pub_vel.publish(cmd_lim);
	}
}

void limiter::spin()
{
	ros::Rate loop_rate(20);

	watchdog_cloud = watchdog_odom = watchdog_vel = -1;
	watchdog_init = 10;
	geometry_msgs::Twist stop;
	int cnt = 0;

	stop.linear.x = stop.linear.y = stop.linear.z = 0;
	stop.angular.x = stop.angular.y = stop.angular.z = 0;
	cmd = stop;

	while(ros::ok())
	{
		if(disabled){
			safety_limiter::Status status;
			status.header.stamp = ros::Time::now();
			status.status = safety_limiter::Status::DISABLED;
			pub_status.publish(status);
			pub_vel.publish(cmd);
		}else{
			if(watchdog_cloud > 0 && watchdog_odom > 0) check();
			watchdog_vel --;
			watchdog_cloud --;
			watchdog_odom --;
			if(watchdog_odom < 0 || watchdog_cloud < 0)
			{
				if( watchdog_odom == 0 ) ROS_ERROR("odom timedout");
				if( watchdog_cloud == 0 ) ROS_ERROR("cloud timedout");
				pub_vel.publish(stop);
			}
			if(cnt % 5 == 0)
			{
				footprint.header.stamp = ros::Time::now();
				pub_footprint.publish(footprint);
			}
			cnt ++;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "safety_limiter");

	limiter lim;
	lim.spin();

	return 0;
}

