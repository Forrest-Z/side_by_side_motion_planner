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
geometry_msgs::Vector3Stamped _speedKmph;
static ros::Publisher _speedMpsPublisher;

#define NODENAME_DEFAULT ("kmph2mps")

static void speedKmphCallback(const geometry_msgs::Vector3Stamped::ConstPtr& input)
{
  _speedKmph = *input;
}

void convertUnit( double coefficient )
{
  geometry_msgs::Vector3Stamped speedMps;  
  speedMps = _speedKmph;

  speedMps.vector.x = coefficient*_speedKmph.vector.x;
  speedMps.vector.y = coefficient*_speedKmph.vector.y;
  speedMps.vector.z = coefficient*_speedKmph.vector.z;
  
  _speedMpsPublisher.publish( speedMps );
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
  
  std::string speedKmphTopicName;
  nh.param<std::string>( "speedKmphTopicName", speedKmphTopicName, "/can_data/sp1" );
  std::cout << "speedKmphTopicName: " << speedKmphTopicName.c_str() << std::endl;  

  std::string speedMpsTopicName;
  nh.param<std::string>( "speedMpsTopicName", speedMpsTopicName, "/vehicle_speed" );
  std::cout << "speedMpsTopicName: " << speedMpsTopicName.c_str() << std::endl;  

  double coefficient;
  nh.param<double>( "coefficient", coefficient, 1/3.6 );
  std::cout << "coefficient: " << coefficient << std::endl;  	
  
  //double tmp;
  //nh.param<double>( "tmp", tmp, 1.0 );
  //std::cout << "tmp: " << tmp << std::endl;  

  //std::string tmp;
  //nh.param<std::string>( "tmp", tmp, "/tmp" );
  //std::cout << "tmp: " << tmp.c_str() << std::endl;  
  //=================================

  //define publisher
  _speedMpsPublisher = nh.advertise<geometry_msgs::Vector3Stamped>(speedMpsTopicName, queSize);
  
  //copy to global memory
  //...
  
  //change unit
    
  //define subsciber  
  ros::Subscriber speedKmphSubscriber = nh.subscribe(speedKmphTopicName, queSize, speedKmphCallback);
      
  //copy to global memory
  
  //starting ros loop    
  ros::Rate loopRate(samplingTimeHz); // by Hz

  while(ros::ok()){
    
    ros::spinOnce();
    
    convertUnit(coefficient);
    
    loopRate.sleep();
  }

}




