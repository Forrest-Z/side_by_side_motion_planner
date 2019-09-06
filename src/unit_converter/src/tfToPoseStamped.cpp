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
static ros::Publisher _poseStampedPublisher;

std::string _tf1TopicName;
std::string _tf2TopicName;

#define NODENAME_DEFAULT ("tfToPoseStamped")
#define TF_WAIT_DURATION_SEC (1e-02)


void convertUnit(void)
{
  static tf::TransformListener tfListener;
  tf::StampedTransform transform;
  try{
    tfListener.lookupTransform( _tf1TopicName, _tf2TopicName, ros::Time(0), transform);
  }
  catch(tf::TransformException &ex){
    ROS_ERROR("%s", ex.what());
  }  
  
  geometry_msgs::PoseStamped poseStamped;	

  poseStamped.header.stamp = ros::Time::now();
  poseStamped.header.frame_id = _tf1TopicName;
  
  poseStamped.pose.position.x = transform.getOrigin().getX();
  poseStamped.pose.position.y = transform.getOrigin().getY();
  poseStamped.pose.position.z = transform.getOrigin().getZ();
  
  poseStamped.pose.orientation.x = transform.getRotation().getX();
  poseStamped.pose.orientation.y = transform.getRotation().getY();
  poseStamped.pose.orientation.z = transform.getRotation().getZ();
  poseStamped.pose.orientation.w = transform.getRotation().getW();
      
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
    
  std::string tf1TopicName;
  nh.param<std::string>( "tf1TopicName", tf1TopicName, "/map" );
  std::cout << "tf1TopicName: " << tf1TopicName.c_str() << std::endl;  

  std::string tf2TopicName;
  nh.param<std::string>( "tf2TopicName", tf2TopicName, "/odom" );
  std::cout << "tf2TopicName: " << tf2TopicName.c_str() << std::endl;  

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
  _tf1TopicName = tf1TopicName;
  _tf2TopicName = tf2TopicName;
  
  //change unit
    
  //define subsciber  

      
  //copy to global memory
  
  //starting ros loop    
  ros::Rate loopRate(samplingTimeHz); // by Hz

  while(ros::ok()){
    
    ros::spinOnce();
    
    convertUnit();
    
    loopRate.sleep();
  }

}




