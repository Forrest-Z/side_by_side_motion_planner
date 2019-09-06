

#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <trajectory_tracker/TrajectoryTrackerStatus.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <trajectory_tracker/TrajectoryPath.h>


static const double MAX_ADMISIBLE_VEL     = 23.0;  //82 km/h
static const double MAX_ADMISIBLE_STEERING_ANG = 0.6109; //  35 deg
static const double MAX_ADMISIBLE_ACC     = 10.0;
static const double DEFAULT_DIST_BETWEEN_FRONT_BACK_WHEEL = 2.7;
//static const double DEFAULT_LOW_PASS_FILTER_COEFFICIENT = 0.5 ;
static const bool  DEFAULT_USE_SIMULATOR = true ;
static const double MAX_DRIVING_WHEEL_W = 6.2831;//360;
static const double TIRE_TO_DRIVINGWHEEL_RATIO = 15.52;
static const double MAX_ADMISIBLE_ANG_VEL = 0.8;


class tracker
{
public:
	tracker();
	~tracker();
	void spin();
private:
	std::string topicPath;
	std::string topicCmdVel;
	std::string topicOdom;
	std::string frameRobot;
	double hz;
	double lookForward;
	double curvForward;
	double k[3];
	double d_lim;
	double d_stop;
	double vel[2];
        double max_admisible_vel;
        double max_admisible_ang_vel;
  double max_admisible_steering_ang;
	double acc[2];
        double max_admisible_acc;
	double set_w;
  double prev_set_steering_angle;
	double set_v;
	double dec;
  //double rotate_ang;
	double angFactor;
  //double swDist;
	double goalToleranceDist;
	double goalToleranceAng;
	double stopToleranceDist;
	double stopToleranceAng;
	double noPosCntlDist;
	int pathStep;
	int pathStepDone;
	bool outOfLineStrip;
    bool allowBackward;
	bool limitVelByAvel;
  double L_baseline; 
        bool use_logged_velocity ;
  double low_pass_filter_coefficient;
  double max_driving_wheel_w;
  double max_tire_w ;
  double omega_dot ;	
	int error_cnt;
  double tire_to_drivingwheel_ratio ;

	ros::Subscriber subPath;
	ros::Subscriber subOdom;
	ros::Subscriber subVel;
        ros::Subscriber subAcc;
        ros::Subscriber subAngVel;
	ros::Publisher pubVel;
	ros::Publisher pubStatus;
	ros::Publisher pubTracking;

	ros::NodeHandle nh;
	tf::TransformListener tf;

  //nav_msgs::Path path;
  trajectory_tracker::TrajectoryPath pose_vel_array;
	nav_msgs::Odometry odom;

  //void cbPath(const nav_msgs::Path::ConstPtr& msg);
  void cbPath(const trajectory_tracker::TrajectoryPath::ConstPtr& msg);

	void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
	void cbSpeed(const std_msgs::Float32::ConstPtr& msg);
        void cbAcc(const std_msgs::Float32::ConstPtr& msg);
        void cbAngVel(const std_msgs::Float32::ConstPtr& msg);

	void control();
};

template<typename T>
class average
{
public:
	average():
		sum()
	{
		num = 0;
	};
	void operator +=(const T &val)
	{
		sum += val;
		num ++;
	};
	operator T()
	{
		if(num == 0) return 0;
		return sum / num;
	};
private:
	T sum;
	int num;
};

tracker::tracker() :
	set_w(0.0),
	set_v(0.0),
	prev_set_steering_angle(0.0),
	nh("~")
{
	nh.param("frame_robot", frameRobot, std::string("base_link"));
	nh.param("path", topicPath, std::string("path"));
	nh.param("odom", topicOdom, std::string("/odom"));
	nh.param("cmd_vel", topicCmdVel, std::string("cmd_vel"));
	nh.param("hz", hz, 50.0);
	nh.param("look_forward", lookForward, 0.5);
	nh.param("curv_forward", curvForward, 0.5);
	nh.param("k_dist", k[0], 1.0);
	nh.param("k_ang", k[1], 1.0);
	nh.param("k_avel", k[2], 1.0);
	nh.param("k_dcel", dec, 0.2);
	nh.param("dist_lim", d_lim, 0.5);
	nh.param("dist_stop", d_stop, 2.0);
	//nh.param("rotate_ang", rotate_ang, M_PI / 4);
	nh.param("max_vel", max_admisible_vel, 0.5);
	nh.param("max_steering_ang", max_admisible_steering_ang, 1.0);	
	nh.param("max_acc", max_admisible_acc, 1.0);
	nh.param("max_driving_wheel_w", max_driving_wheel_w, MAX_DRIVING_WHEEL_W);
	nh.param( "tire_to_drivingwheel_ratio", tire_to_drivingwheel_ratio, TIRE_TO_DRIVINGWHEEL_RATIO );
	nh.param("max_angacc", acc[1], 2.0);
	nh.param("path_step", pathStep, 1);
	nh.param("distance_angle_factor", angFactor, 0.0);
	//nh.param("switchback_dist", swDist, 0.3);
	nh.param("goal_tolerance_dist", goalToleranceDist, 0.2);
	nh.param("goal_tolerance_ang", goalToleranceAng, 0.1);
	nh.param("stop_tolerance_dist", stopToleranceDist, 0.1);
	nh.param("stop_tolerance_ang", stopToleranceAng, 0.05);
	//nh.param("no_position_control_dist", noPosCntlDist, 0.0);
	nh.param("allow_backward", allowBackward, true);
	nh.param("limit_vel_by_avel", limitVelByAvel, false);
	nh.param("L_baseline", L_baseline, DEFAULT_DIST_BETWEEN_FRONT_BACK_WHEEL);
	nh.param("use_logged_velocity", use_logged_velocity , true);
	//nh.param("low_pass_filter_coef", low_pass_filter_coefficient , DEFAULT_LOW_PASS_FILTER_COEFFICIENT);
	//std::cerr << "\nfilter_coeff: " << low_pass_filter_coefficient <<"\n\n";

	if ( max_admisible_vel > MAX_ADMISIBLE_VEL )
	  max_admisible_vel = MAX_ADMISIBLE_VEL;
	vel[0] = max_admisible_vel;

	nh.param("max_angvel", max_admisible_ang_vel, 1.0);
	if ( max_admisible_ang_vel > MAX_ADMISIBLE_ANG_VEL )
	  max_admisible_ang_vel = MAX_ADMISIBLE_ANG_VEL;
	vel[1] = max_admisible_ang_vel;
	
	if ( max_admisible_steering_ang > MAX_ADMISIBLE_STEERING_ANG )
	  max_admisible_steering_ang = MAX_ADMISIBLE_STEERING_ANG;

	
	if ( max_admisible_acc > MAX_ADMISIBLE_ACC )
	  max_admisible_acc = MAX_ADMISIBLE_ACC;
	acc[0] = max_admisible_acc;
	max_tire_w = max_driving_wheel_w / tire_to_drivingwheel_ratio ;//* M_PI / 180 ;

	subPath = nh.subscribe(topicPath, 2, &tracker::cbPath, this);
	subOdom = nh.subscribe(topicOdom, 20, &tracker::cbOdom, this);
	subVel = nh.subscribe("/trajectory_tracker/max_vel", 20, &tracker::cbSpeed, this);
	subAngVel = nh.subscribe("/trajectory_tracker/max_ang_vel", 20, &tracker::cbAngVel, this);
	subAcc = nh.subscribe("/trajectory_tracker/max_acc", 20, &tracker::cbAcc, this);
	pubVel = nh.advertise<geometry_msgs::Twist>(topicCmdVel, 10);
	pubStatus = nh.advertise<trajectory_tracker::TrajectoryTrackerStatus>("status", 10);
	pubTracking = nh.advertise<geometry_msgs::PoseStamped>("tracking", 10);

	std::cerr << "\nTrajectory_Tracker_Params->\n Max_vel: " << max_admisible_vel << " ";
	std::cerr << "steering_ang: " << max_admisible_steering_ang << " ";
	std::cerr << "max_acc: " << max_admisible_acc ;
	std::cerr << " max_driving_wheel_w: " << max_driving_wheel_w << " tire_wheel_ratio: " << tire_to_drivingwheel_ratio ;
	std::cerr << "\n\n";			
}

tracker::~tracker()
{
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;

	pubVel.publish(cmd_vel);
}

float dist2d(geometry_msgs::Point &a, geometry_msgs::Point &b)
{
	return sqrtf(powf(a.x-b.x,2) + powf(a.y-b.y,2));
}
float len2d(geometry_msgs::Point &a)
{
	return sqrtf(powf(a.x,2) + powf(a.y,2));
}
float len2d(geometry_msgs::Point a)
{
	return sqrtf(powf(a.x,2) + powf(a.y,2));
}
float curv3p(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	float ret;
	ret = 2 * (a.x*b.y + b.x*c.y + c.x*a.y - a.x*c.y - b.x*a.y - c.x*b.y);
	ret /= sqrtf( (powf(b.x-a.x, 2) + powf(b.y-a.y, 2)) * (powf(b.x-c.x, 2) + powf(b.y-c.y, 2)) * (powf(c.x-a.x, 2) + powf(c.y-a.y, 2)) );

	return ret;
}
float cross2d(geometry_msgs::Point &a, geometry_msgs::Point &b) 
{
	return a.x*b.y - a.y*b.x;
}
float cross2d(geometry_msgs::Point a, geometry_msgs::Point b) 
{
	return a.x*b.y - a.y*b.x;
}
float dot2d(geometry_msgs::Point &a, geometry_msgs::Point &b) 
{
	return a.x*b.x + a.y*b.y;
}
float dot2d(geometry_msgs::Point a, geometry_msgs::Point b) 
{
	return a.x*b.x + a.y*b.y;
}
geometry_msgs::Point point2d(float x, float y)
{
	geometry_msgs::Point ret;
	ret.x = x;
	ret.y = y;
	return ret;
}
geometry_msgs::Point sub2d(geometry_msgs::Point &a, geometry_msgs::Point &b)
{
	geometry_msgs::Point ret;
	ret.x = a.x - b.x;
	ret.y = a.y - b.y;
	return ret;
}
float sign(float a)
{
	if(a < 0) return -1;
	return 1;
}
float dist2d_line(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	return (cross2d(sub2d(b, a), sub2d(c, a)) / dist2d(b, a));
}
float dist2d_linestrip(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	if(dot2d(sub2d(b, a), sub2d(c, a) ) <= 0) return dist2d(c, a);
	if(dot2d(sub2d(a, b), sub2d(c, b) ) <= 0) return -dist2d(c, b) - 0.005;
	return fabs( dist2d_line(a, b, c) );
}
geometry_msgs::Point projection2d(geometry_msgs::Point &a, geometry_msgs::Point &b, geometry_msgs::Point &c)
{
	float r = dot2d(sub2d(b, a), sub2d(c, a)) / pow(len2d(sub2d(b, a)), 2);
	geometry_msgs::Point ret;
	ret.x = b.x*r + a.x*(1-r);
	ret.y = b.y*r + a.y*(1-r);
	return ret;
}

void tracker::cbSpeed(const std_msgs::Float32::ConstPtr& msg)
{
  if ( max_admisible_vel > msg->data )
    vel[0] = msg->data;
  else
    vel[0] = max_admisible_vel ;
  
  std::cerr << "max_vel: " << vel[0] << "\n";
}

void tracker::cbAcc(const std_msgs::Float32::ConstPtr& msg)
{
  if ( max_admisible_acc > msg->data )
    acc[0] = msg->data;
  else
    acc[0] = max_admisible_acc ;
  
  std::cerr << "max_acc: " << acc[0] << "\n";
}

void tracker::cbAngVel(const std_msgs::Float32::ConstPtr& msg)
{
  if ( max_admisible_steering_ang > msg->data )
    vel[1] = msg->data;
  else
    vel[1] = max_admisible_steering_ang ;
  
  std::cerr << "max_ang_vel: " << vel[1] << "\n";
}



void tracker::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom = *msg;
	//std::cerr << odom.twist.twist.linear.x << " " << odom.twist.twist.angular.z << "\n";
}

//void tracker::cbPath(const nav_msgs::Path::ConstPtr& msg)
void tracker::cbPath(const trajectory_tracker::TrajectoryPath::ConstPtr& msg)
{
  //path = *msg;
  //std::cerr << "\n" << *msg;
  
	pose_vel_array = *msg ;
	
	//path.header = pose_vel_array.header;
	//path.poses = pose_vel_array.pose;

	//auto i = path.poses.begin();
	auto i = pose_vel_array.poses.begin();
	//for(auto j = path.poses.begin(); j != path.poses.end();)
	for(auto j = pose_vel_array.poses.begin(); j != pose_vel_array.poses.end();)
	{
		if(i != j && dist2d((*i).pose.position, (*j).pose.position) < 0.01)
		{
			j = pose_vel_array.poses.erase(j);
			continue;
		}
		i = j;
		//std::cerr << "\nvel" << (*i).twist.linear.x << " ang_vel " << (*i).twist.angular.z ;
		//std::cerr << "\nvel" << (*i).twist.linear.x << " ang_vel " << (*i).twist.angular.z << " header: " << pose_vel_array.header;;
		j ++;
	}
	pathStepDone = 0;
	//std::cerr << pose_vel_array << "\n Inside Ackermann trajectory tracker  \n\n";
}

void tracker::spin()
{
	ros::Rate loop_rate(hz);

	while(ros::ok())
	{
		control();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void tracker::control()
{
	trajectory_tracker::TrajectoryTrackerStatus status;
	status.header.stamp = ros::Time::now();
	//status.header.seq = path.header.seq;
	status.header.seq = pose_vel_array.header.seq;
	status.distance_remains = 0.0;
	status.angle_remains = 0.0;

	//std::cerr << pose_vel_array.header << "\n Inside Ackermann trajectory tracker" << pose_vel_array.poses.size() <<"  \n\n";

	//if(path.header.frame_id.size() == 0 || path.poses.size() < 3)
	if(pose_vel_array.header.frame_id.size() == 0 || pose_vel_array.poses.size() < 3)
	{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0;
		pubVel.publish(cmd_vel);
		status.status = trajectory_tracker::TrajectoryTrackerStatus::NO_PATH;
		//std::cerr << "\n No Path " << pose_vel_array.header ; //.size() << " " << pose_vel_array.poses.size();
		pubStatus.publish(status);
		return;
	}					
	// Transform
	nav_msgs::Path lpath;
	trajectory_tracker::TrajectoryPath temp_vel;
	//lpath.header = path.header;
	lpath.header = pose_vel_array.header;
	tf::StampedTransform transform;
	try
	{
		ros::Time now = ros::Time(0);
		//tf.waitForTransform(frameRobot, path.header.frame_id, now, ros::Duration(0.05));
		//tf.lookupTransform(frameRobot, path.header.frame_id, now, transform);

		
		//std::cerr << pose_vel_array.header.frame_id << " Inside Ackermann trajectory tracker" << pose_vel_array.poses.size() << " framerobot:" << frameRobot <<  "  \n\n";


		
		tf.waitForTransform(frameRobot, pose_vel_array.header.frame_id, now, ros::Duration(0.05));
		tf.lookupTransform (frameRobot, pose_vel_array.header.frame_id, now, transform);
		
	
		if(fabs((ros::Time::now() - transform.stamp_).toSec()) > 0.1)
		{
			if(error_cnt % 16 == 0)
				ROS_ERROR("Timestamp of the transform is too old %f %f", ros::Time::now().toSec(), transform.stamp_.toSec());
			error_cnt ++;
		}
		else
		{
			error_cnt = 0;
		}


		
				
		//for(int i = 0; i < (int)path.poses.size(); i += pathStep)
		for(int i = 0; i < (int)pose_vel_array.poses.size(); i += pathStep)
		{
		  geometry_msgs::PoseStamped temp;
		  geometry_msgs::PoseStamped pose;
		  temp.header = pose_vel_array.header ;
		  temp.pose.position = pose_vel_array.poses[i].pose.position;
		  temp.pose.orientation = pose_vel_array.poses[i].pose.orientation;
		  //tf.transformPose(frameRobot, now, path.poses[i], path.header.frame_id, pose);
		  tf.transformPose(frameRobot, now, temp, pose_vel_array.header.frame_id, pose);
		  lpath.poses.push_back(pose);
		  temp_vel.poses.push_back (pose_vel_array.poses[i] );
		}


	}

	
	catch (tf::TransformException &e)
	{
		ROS_WARN("TF exception: %s", e.what());
		status.status = trajectory_tracker::TrajectoryTrackerStatus::NO_PATH;
		pubStatus.publish(status);
		return;
	}
	

	float minDist = 10000.0;
	int iclose = -1;
	geometry_msgs::Point origin; 

	//Looking Forward with Ackermann Model
	double kappa = tan( set_w/2 ) / L_baseline ;
	
	double C0 = cos(set_w * lookForward / 2.0);
	double S0 = sin(set_w * lookForward / 2.0);
	double dv = set_v * lookForward;
	
	if( set_w == 0)  {// Curvature	  
	  origin.x = dv * C0  ;
	  origin.y = dv * S0  ; 
	}
	else	  {

	  double dw = dv * kappa ;
	  double theta = dw * lookForward /2;

	  double C1 = cos ( theta ) ;
	  double S1 = sin ( theta ) ;
	  double R  = 1 / kappa ;
	  
	  origin.x = R * ( -S0 + S1 ) ;
	  origin.y = R * (  C0 - C1 ) ;

	}


	
	//origin.x = cos(set_w * lookForward / 2.0) * set_v * lookForward;
	//origin.y = sin(set_w * lookForward / 2.0) * set_v * lookForward;


	// Find nearest line strip
	outOfLineStrip = false;
	float distancePath = 0;
	for(int i = 1; i < (int)lpath.poses.size(); i ++)
	{
		distancePath += dist2d(lpath.poses[i-1].pose.position, lpath.poses[i].pose.position);
	}
	float distancePathSearch = 0;
	for(int i = pathStepDone; i < (int)lpath.poses.size(); i ++)
	{
		if(i < 1) continue;
		distancePathSearch += dist2d(lpath.poses[i-1].pose.position, lpath.poses[i].pose.position);
		if(dist2d(origin, lpath.poses[i].pose.position) < 0.05 &&
				i < (int)lpath.poses.size() - 1) continue;
		float d = dist2d_linestrip(lpath.poses[i-1].pose.position, lpath.poses[i].pose.position, origin);
		if(fabs(d) <= fabs(minDist))
		{
			minDist = d;
			iclose = i;
			
		}
		if(pathStepDone > 0 && distancePathSearch > 1.0) break;
		//std::cerr << "mm\n";
	}

	
	
	//std::cerr << "\nminDist " << minDist;
	if(iclose < 0)
	{
	  geometry_msgs::Twist cmd_vel;
	  cmd_vel.linear.x = 0;
	  cmd_vel.angular.z = 0;
	  pubVel.publish(cmd_vel);
	  ROS_WARN("failed to find nearest node");
	  status.status = trajectory_tracker::TrajectoryTrackerStatus::NO_PATH;
	  pubStatus.publish(status);
	  return;
	}

	//Get here the velocities and write down the conditions. by Luis
	//std::cerr << "\nvel: " << lpath.poses[iclose-1].pose.orientation.x;



	// Signed distance error
	float dist = dist2d_line(lpath.poses[iclose-1].pose.position, lpath.poses[iclose].pose.position, origin);
	float _dist = dist;
	if(iclose == 0)
	{
		_dist = -dist2d(lpath.poses[iclose].pose.position, origin);
	}
	//if(iclose + 1 >= (int)path.poses.size())
	if(iclose + 1 >= (int)pose_vel_array.poses.size())
	{
		_dist = -dist2d(lpath.poses[iclose].pose.position, origin);
	}
	
	// Angular error
	geometry_msgs::Point vec = sub2d(lpath.poses[iclose].pose.position, lpath.poses[iclose - 1].pose.position);
	float angle = -atan2(vec.y, vec.x);
	float anglePose;
	if(allowBackward) anglePose = tf::getYaw(lpath.poses[iclose].pose.orientation);
	else anglePose = -angle;
	float signVel = 1.0;
	if(cos(-angle) * cos(anglePose) + sin(-angle) * sin(anglePose) < 0)
	{
		signVel = -1.0;
		angle = angle + M_PI;
		if(angle > M_PI) angle -= 2.0 * M_PI;
	}
	// Curvature
	average<float> curv;
	geometry_msgs::Point posLine = projection2d(lpath.poses[iclose-1].pose.position, lpath.poses[iclose].pose.position, origin);
	int local_goal = lpath.poses.size() - 1;
	float remainLocal = 0;
	remainLocal = dist2d(posLine, lpath.poses[iclose].pose.position);
	for(int i = iclose - 1; i < (int)lpath.poses.size() - 1; i ++)
	{
		if(i > 2)
		{
			geometry_msgs::Point vec = sub2d(lpath.poses[i].pose.position, 
					lpath.poses[i-1].pose.position);
			float angle = atan2(vec.y, vec.x);
            float anglePose;
            if(allowBackward) anglePose = tf::getYaw(lpath.poses[i+1].pose.orientation);
            else anglePose = angle;
			float signVel_req = cos(angle) * cos(anglePose) + sin(angle) * sin(anglePose);
			if(signVel * signVel_req < 0)
			{
				// Stop read forward if the path switched back
				local_goal = i;
				break;
			}
			if(i > iclose)
				remainLocal += dist2d(lpath.poses[i-1].pose.position, lpath.poses[i].pose.position);
		}
	}
	for(int i = iclose - 1; i < local_goal; i ++)
	{
		if(i > 2)
		{
			curv += curv3p(lpath.poses[i-2].pose.position, lpath.poses[i-1].pose.position, lpath.poses[i].pose.position);
		}
		if(dist2d(lpath.poses[i].pose.position, 
					lpath.poses[local_goal].pose.position) < 0.05) break;
		if(dist2d(lpath.poses[i].pose.position, posLine) > curvForward) break;
	}
	float remain;
	remain = dist2d(origin, lpath.poses.back().pose.position);
	if(minDist < 0 && iclose == local_goal) outOfLineStrip = true;
	if(outOfLineStrip)
	{
		remain = -remain;
		remainLocal = -remainLocal;
	}
	if(distancePath < noPosCntlDist) remain = remainLocal = 0;
	//fprintf(stderr,"%d %d   %0.3f  %+0.3f %+0.3f  %f  %f  sv %f\n",outOfLineStrip, iclose, distancePath, remain,remainLocal,minDist, angle, signVel);
	//printf("d=%.2f, th=%.2f, curv=%.2f\n", dist, angle, (float)curv);
    while(angle < -M_PI) angle += 2.0*M_PI;
    while(angle > M_PI) angle -= 2.0*M_PI;

	status.distance_remains = remain;
	status.angle_remains = angle;

	float dt = 1/hz;
	float _v = set_v;
	float _w = set_w;

	
	//iclose  Luis
	if( use_logged_velocity == true && temp_vel.poses[iclose].twist.linear.x != 0.0)
	  if ( temp_vel.poses[iclose].twist.linear.x  < max_admisible_vel )
	    vel[0] = temp_vel.poses[ iclose ].twist.linear.x ;
	  else
	    vel[0] = max_admisible_vel ;

	/*
	if ( pose_vel_array.poses[ iclose ].twist.linear.x  < max_admisible_vel )
	  vel[0] = pose_vel_array.poses[ iclose ].twist.linear.x ;
	else
	  vel[0] = max_admisible_vel ;
	 */	
	//std::cerr << "\nmax_vel: " << vel[0] << "m/s  " ;//<< vel[0]*3.6 << "km/h";




	/*  THERE CAN NOT BE STOP AND ROTATE CONDITION IN ACKERMANN STEERING so this loop was erased by Luis
	// Stop and rotate
	if((fabs(rotate_ang) < M_PI && cos(rotate_ang) > cos(angle)) ||
		fabs(status.distance_remains) < stopToleranceDist)
	{
		w = -sign(angle) * sqrtf(fabs(2 * angle * acc[1] * 0.9));
		v = 0;
		if(v > vel[0]) v = vel[0];
		else if(v < -vel[0]) v = -vel[0];
		if(v > _v + dt*acc[0]) v = _v + dt*acc[0];
		else if(v < _v - dt*acc[0]) v = _v - dt*acc[0];
		if(w > vel[1]) w = vel[1];
		else if(w < -vel[1]) w = -vel[1];
		if(w > _w + dt*acc[1]) w = _w + dt*acc[1];
		else if(w < _w - dt*acc[1]) w = _w - dt*acc[1];
		//ROS_WARN("Rotate");
	}
	
	else
	{
*/

	
		// Control
	if(dist < -d_lim) 
	  dist = -d_lim;
	else if(dist > d_lim) 
	  dist = d_lim;

	set_v = sign(remainLocal) * signVel * sqrtf(fabs(2 * remainLocal * acc[0] * 0.95));

	if(set_v > vel[0]) 
	  set_v = vel[0];
      	else if(set_v < -vel[0]) 
	  set_v = -vel[0];
	
	if(set_v > _v + dt*acc[0]) 
	  set_v = _v + dt*acc[0];
		
	else if(set_v < _v - dt*acc[0]) 
	  set_v = _v - dt*acc[0];

	float wref = fabs(set_v) * curv;

	if(limitVelByAvel)
	  {
	    //std::cerr << "\n limit vel by avel 0"  ;
	    if(fabs(wref) > vel[1])
	      {
		//std::cerr << "\n limit vel by avel 1"  ;
		set_v = sign(set_v) * fabs(vel[1] / curv);
		if(set_v > vel[0]) set_v = vel[0];
		else if(set_v < -vel[0]) set_v = -vel[0];
		if(set_v > _v + dt*acc[0]) set_v = _v + dt*acc[0];
		else if(set_v < _v - dt*acc[0]) set_v = _v - dt*acc[0];
	      }
	  }

	
	//set_w += dt * (-dist*k[0] -angle*k[1] -(set_w - wref)*k[2]);
	set_w += dt * (-dist*k[0] -angle*k[1] -(odom.twist.twist.angular.z - wref)*k[2]);

	
	
	//set_w = set_w * (1-low_pass_filter_coefficient) + prev_set_w*(low_pass_filter_coefficient);

	//Add condition for angular velocity

	//if ( (set_w - prev_w)/time )


	
	
	if(set_w > vel[1]) 
	  set_w = vel[1];
	else if(set_w < -vel[1]) 
	  set_w = -vel[1];
	
	if(set_w > _w + dt*acc[1]) 
	  set_w = _w + dt*acc[1];
	else if(set_w < _w - dt*acc[1]) 
	  set_w = _w - dt*acc[1];

	if(!std::isfinite(set_v)) 
	  set_v = 0;
	if(!std::isfinite(set_w)) 
	  set_w = 0;




	
	
	//std::cerr << "\n comp_dist: "<< _dist;
	// Too far from given path
	if(fabs(_dist) > d_stop)
	  {
	    std::cerr << "\n Too far from given path " << d_stop  << " comp_dist: "<< _dist;
	    geometry_msgs::Twist cmd_vel;
	    cmd_vel.linear.x = 0;
	    cmd_vel.angular.z = 0;
	    pubVel.publish(cmd_vel);
	    ROS_WARN("Far from given path %f", d_stop);
	    status.status = trajectory_tracker::TrajectoryTrackerStatus::FAR_FROM_PATH;
	    pubStatus.publish(status);
	    return;
	  }
	//}
	
	geometry_msgs::Twist cmd_vel;
	if(fabs(status.distance_remains) < stopToleranceDist &&
	   fabs(status.angle_remains) < stopToleranceAng)
	  {
	    set_v = 0;
	    set_w = 0;
	    
	  }


	cmd_vel.linear.x = set_v;


	double k;
	double steering_angle;
	
	
	
	if ( set_v == 0.0){//odom.twist.twist.linear.x == 0){
	  set_w =0;
	  steering_angle=0;
	}
	else{
	  k = set_w / set_v ;
	  //k = odom.twist.twist.angular.z / odom.twist.twist.linear.x ;
	  steering_angle = atan (k * L_baseline);
	}


	//if (  (steering_angle - prev_set_steering_angle) < 0.5*M_PI/180)
	//steering_angle = prev_set_steering_angle;

	//std::cerr << ros::Time::now();
	//std::cerr << " control_max_vel " << set_v << " " << set_v*3.6 << " km/h"; 	
	//std::cerr << " ang_vel " << steering_angle << " " << steering_angle*180/M_PI << " deg\n"; 





	/*
	//Condition for angular velocity in simulation	
	omega_dot =  steering_angle - prev_set_steering_angle;	


	if ( omega_dot > max_tire_w *dt ){
	  std::cerr << "\nDd: " << omega_dot*180/M_PI << " w^d" << omega_dot ;
	  omega_dot = max_tire_w * dt ;

	}
	else if ( omega_dot < -max_tire_w * dt){
	  std::cerr << "\nDd: " << omega_dot*180/M_PI << " w^d" << omega_dot ;
	  omega_dot = -max_tire_w * dt ;
	}

	set_w = prev_set_steering_angle + omega_dot ;

	if ( set_w > max_admisible_steering_ang )
	  set_w = max_admisible_steering_ang;
	*/
	
	cmd_vel.angular.z = steering_angle ;// set_w;
	pubVel.publish(cmd_vel);
	prev_set_steering_angle = steering_angle;//set_w;

	status.status = trajectory_tracker::TrajectoryTrackerStatus::FOLLOWING;
	if(fabs(status.distance_remains) < goalToleranceDist &&
	   fabs(status.angle_remains) < goalToleranceAng)
	  {
	    status.status = trajectory_tracker::TrajectoryTrackerStatus::GOAL;
	  }
	pubStatus.publish(status);
	geometry_msgs::PoseStamped tracking;
	tracking.header = status.header;
	tracking.header.frame_id = frameRobot;
	tracking.pose.position = posLine;
	tracking.pose.orientation = tf::createQuaternionMsgFromYaw(-angle);
	pubTracking.publish(tracking);

	pathStepDone = iclose;
	if(pathStepDone < 0) pathStepDone = 0;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ackermann_trajectory_tracker");

	tracker track;
	track.spin();

	return 0;
}
