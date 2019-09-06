#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <vector>

using namespace std;

//define global values
nav_msgs::Odometry _odometry;
static ros::Publisher _poseStampedPublisher;

#define NODENAME_DEFAULT ("odometryToPoseStamped")

static void odometryCallback(const nav_msgs::Odometry::ConstPtr& input)
{
  _odometry = *input;
}

void convertUnit(void)
{
  geometry_msgs::PoseStamped poseStamped;	
  poseStamped.pose = _odometry.pose.pose;
  
  _poseStampedPublisher.publish( poseStamped );
}

int main (int argc, char *argv[])
{
  //init ros loop 
  ros::init(argc, argv, NODENAME_DEFAULT);
  ros::NodeHandle nh("~");

  //=== input parameters =======================
  std::cout << "nodeName:   " << ros::this_node::getName()  << std::endl;    
  
  int samplingTimeHz;
  nh.param<int>( "samplingTimeHz", samplingTimeHz, 100 );
  std::cout << "samplingTimeHz: " << samplingTimeHz << std::endl;  

  int queSize;
  nh.param<int>( "queSize", queSize, 10 );
  std::cout << "queSize: " << queSize << std::endl;  
  
  std::string odometryTopicName;
  nh.param<std::string>( "odometryTopicName", odometryTopicName, "/odom" );
  std::cout << "odometryTopicName: " << odometryTopicName.c_str() << std::endl;  

  std::string poseStampedTopicName;
  nh.param<std::string>( "poseStampedTopicName", poseStampedTopicName, "/filtered_ndt_current_pose" );
  std::cout << "poseStampedTopicName: " << poseStampedTopicName.c_str() << std::endl;  
  
  //double tmp;
  //nh.param<double>( "tmp", tmp, 1.0 );
  //std::cout << "tmp: " << tmp << std::endl;  

  //std::string tmp;
  //nh.param<std::string>( "tmp", tmp, "/tmp" );
  //std::cout << "tmp: " << tmp.c_str() << std::endl;  
  //=================================

  //define publisher
  _poseStampedPublisher = nh.advertise<geometry_msgs::PoseStamped>(poseStampedTopicName, queSize);
  
  //copy to global memory
  //...
  
  //change unit
    
  //define subsciber  
  ros::Subscriber odometrySubscriber = nh.subscribe(odometryTopicName, queSize, odometryCallback);
      
  //copy to global memory
  
  //starting ros loop    
  ros::Rate loopRate(samplingTimeHz); // by Hz

  while(ros::ok()){
    
    ros::spinOnce();
    
    convertUnit();
    
    loopRate.sleep();
  }

}




