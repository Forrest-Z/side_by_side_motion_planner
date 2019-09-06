#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

static const std::string DEFAULT_ODOM_FRAME_ID = "odom";
static const std::string DEFAULT_BASE_FRAME_ID = "base_link";

static const double DEFAULT_FRONT_BACK_BASELINE = 0.3;
static const double DEFAULT_INITIAL_POSE_X = 0.0;//added by yoshihara
static const double DEFAULT_INITIAL_POSE_Y = 0.0;//added by yoshihara
static const double DEFAULT_INITIAL_POSE_THETA = 0.0;//added by yoshihara

class OdoSimulatorNode
{
public:
    OdoSimulatorNode();
    void run();

private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  double dv, steering_angle; // desired velocities
  double x, y, theta, k;
  double front_back_baseline ;
  double dw ;
  
  ros::Publisher pub;
  ros::Publisher pubSpeed;//added by Yoshihara
  ros::Publisher pubYawRate;
  ros::Publisher pubAcceleration;
  ros::Publisher pubCurrentPose;
  ros::Publisher pubFilteredNdtCurrentPose;
  
  ros::Subscriber vel_sub;
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
};


OdoSimulatorNode::OdoSimulatorNode():
    dv(0),
    steering_angle(0),
    x(0),
    y(0),
    theta(0)
{
  ros::NodeHandle pnh("~");

  pub = pnh.advertise<nav_msgs::Odometry>("base_pose_ground_truth", 1);
  pubSpeed = pnh.advertise<geometry_msgs::Vector3Stamped>("/vehicle_speed", 1);
  pubYawRate = pnh.advertise<geometry_msgs::Vector3Stamped>("/vehicle_yawrate", 1);
  pubAcceleration = pnh.advertise<geometry_msgs::Vector3Stamped>("/vehicle_acceleration", 1);
  pubCurrentPose = pnh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1);
  pubFilteredNdtCurrentPose = pnh.advertise<geometry_msgs::PoseStamped>("/filtered_ndt_current_pose", 1);
  
  std::string odom_frame_id, base_frame_id;
  std::string command_velocity;
  pnh.param<std::string>("odom_frame_id", odom_frame_id, DEFAULT_ODOM_FRAME_ID);
  pnh.param<std::string>("base_frame_id", base_frame_id, DEFAULT_BASE_FRAME_ID);
  pnh.param<double>("front_back_baseline", front_back_baseline, DEFAULT_FRONT_BACK_BASELINE);

  pnh.param<std::string>("cmd_vel", command_velocity, "cmd_vel");
  
  //added by yoshihara, 170123
  double initialPoseX, initialPoseY, initialPoseTheta;
  pnh.param<double>("initialPoseX", initialPoseX, DEFAULT_INITIAL_POSE_X );
  pnh.param<double>("initialPoseY", initialPoseY, DEFAULT_INITIAL_POSE_Y );
  pnh.param<double>("initialPoseTheta", initialPoseTheta, DEFAULT_INITIAL_POSE_THETA );
  x = initialPoseX;
  y = initialPoseY;
  theta = initialPoseTheta;

  vel_sub = pnh.subscribe(command_velocity, 1, &OdoSimulatorNode::cmdVelCallback, this);

  std::cerr << "\nFront-back wheel baseline: " << front_back_baseline << "\n\n\n";

  odom_trans.header.frame_id = odom_frame_id;
  odom_trans.child_frame_id = base_frame_id;

  odom.header.frame_id = odom_frame_id;
  odom.child_frame_id = base_frame_id;
}


void OdoSimulatorNode::run(){
    ros::Time current_time = ros::Time::now(), last_timestamp = ros::Time::now();
    double delta_time = 0;  
    ros::Rate loop_rate(40);
    
    while (ros::ok())
    {
      

      k = tan(steering_angle) / front_back_baseline ;

    double C0 =cos( theta );
    double S0 =sin( theta );

    if( steering_angle == 0)  // Curvature
    {
        x += dv * C0 * delta_time ;
        y += dv * S0 * delta_time; 
	//alpha = 0 ;
	//theta = theta ;
        //particle.state.a=0;
    }
    else
    {

	dw = dv * k ;
	theta += dw * delta_time ;

	double C1 = cos ( theta ) ;
	double S1 = sin ( theta ) ;
	double R  = 1 / k ;

	x += R * ( -S0 + S1 ) ;
	y += R * (  C0 - C1 ) ;

    }
	//R= L / tan(steering);
	//R = 1 / k;

	//L = distance between front wheel and back wheel

      /*
oid deviceAckermannModel(TrackerParticle & particle, EgoMotion & egomotion)
{
    double c=cos(particle.state.theta);
    double s=sin(particle.state.theta);

    if(particle.state.k==0)
    {
        particle.state.x=particle.state.x+c*particle.state.v*egomotion.dt;
        particle.state.y=particle.state.y+s*particle.state.v*egomotion.dt;
        particle.state.a=0;
    }
    else
    {
        double c0=cos(particle.state.theta+particle.state.a);
        double s0=sin(particle.state.theta+particle.state.a);

        particle.state.omega=particle.state.v*particle.state.k;
        double dtheta=particle.state.omega*egomotion.dt;
        particle.state.theta+=dtheta;

        double c1=cos(particle.state.theta+particle.state.a);
        double s1=sin(particle.state.theta+particle.state.a);
        double R=1/particle.state.k;

        particle.state.x=particle.state.x+R*(-s0+s1);
        particle.state.y=particle.state.y+R*(c0-c1);
    }
}

omega = angular velocity
v = linear velocity
dt = sampling time

a = orientation offset of velocity (cero for us)
k = curvature
R = 


//apha: steering angle
//theta: body angle
x = v * cos(alpha) * cos(theta)
y = v * cos(alpha) * sin(theta)
theta = V/T * sin(alpha)
  



        
        theta += (dw * delta_time);
        x += dv * cos(theta) * delta_time;
        y += dv * sin(theta) * delta_time;

*/
        current_time = ros::Time::now();
        delta_time = current_time.toSec() - last_timestamp.toSec();
        last_timestamp = ros::Time::now();
        
        // Create a quaternion from thetaeta
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        // Update thetae odometry transform data to send to tf
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

        odom.twist.twist.linear.x = dv;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = dw;
        
        // Publish to odom
        pub.publish(odom);
	
        // Publish speed m/s, yawrate rad/sec, acceleration m/s^2;  added by Yoshihara
	geometry_msgs::Vector3Stamped speed;
	speed.vector.x = dv;
	speed.header.stamp = current_time;  
	pubSpeed.publish( speed );

	geometry_msgs::Vector3Stamped yawRate;	
	yawRate.vector.x = dw;
	yawRate.header.stamp = current_time;  	
	pubYawRate.publish( yawRate );

	geometry_msgs::Vector3Stamped acceleration; 
	static double dv_prev = 0;
	acceleration.vector.x = (dv - dv_prev)/delta_time;
	acceleration.header.stamp = current_time;  	
	pubAcceleration.publish( acceleration );
	dv_prev = dv;

	geometry_msgs::PoseStamped currentPose;	
	currentPose.pose = odom.pose.pose;
	pubCurrentPose.publish( currentPose );	
	pubFilteredNdtCurrentPose.publish( currentPose );	
		
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void OdoSimulatorNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_ERROR( "new speed: [%0.2f][%0.2f]", msg->linear.x, msg->angular.z);
    dv = msg->linear.x;
    steering_angle = msg->angular.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ackermann_odo_simulator");
    OdoSimulatorNode odo_simulator;
    odo_simulator.run();

    return 0;
}
