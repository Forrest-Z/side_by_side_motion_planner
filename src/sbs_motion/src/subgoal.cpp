#include <ros/ros.h>
#include "sbs_motion/subgoalList.h"
#include "sbs_motion/sendGoals.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <traversable.hpp>

#define DISTANCE_TOLERANCE 1.0

ros::Publisher vis_pub;
ros::Publisher next_pub;
ros::Publisher goal_pub;

geometry_msgs::PoseArray goalQueue;
visualization_msgs::Marker marker;
std_msgs::ColorRGBA colour;
tf::Pose v1Pose, v2Pose;
int counter=0;
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if(counter)
    return;
  ROS_INFO_STREAM("X:"<<msg->pose.position.x <<"\t Y:" <<msg->pose.position.y);
  goalQueue.poses.push_back(msg->pose);
  marker.points.push_back(msg->pose.position);
  marker.colors.push_back(colour);

  vis_pub.publish(marker);
}

bool send(sbs_motion::sendGoals::Request &req, sbs_motion::sendGoals::Response &res)
{
  if(counter)
    return false;
  marker.type=visualization_msgs::Marker::LINE_STRIP;
  marker.ns="path";
  marker.scale.x=0.05;
  vis_pub.publish(marker);
  goal_pub.publish(goalQueue);
  marker.scale.x = 0.5;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.ns = "goals";
  marker.points.clear();
  marker.colors.clear();
  goalQueue.poses.clear();
  return true;
  
}



void v1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tf::Vector3 v(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  // tf::Vector3 v(0,0,0);
  // tf::Quaternion q(0,0,0,1);
  v1Pose.setOrigin(v);
  v1Pose.setRotation(q);

}

void v2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tf::Vector3 v(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  // tf::Vector3 v(0,0,0);
  // tf::Quaternion q(0,0,0,1);
  v2Pose.setOrigin(v);
  v2Pose.setRotation(q);

}

void marker_update()
{
  marker.colors[counter].r=0.8;
  marker.colors[counter].g=0.3;
  vis_pub.publish(marker);
}

void check_goals(){
  Eigen::Vector2d v1(v1Pose.getOrigin().getX(),v1Pose.getOrigin().getY());
  Eigen::Vector2d v2(v2Pose.getOrigin().getX(),v2Pose.getOrigin().getY());
  Eigen::Vector2d goal(marker.points[counter].x,marker.points[counter].y);
  
  if((v1-goal).norm()<DISTANCE_TOLERANCE||(v2-goal).norm()<DISTANCE_TOLERANCE){
    marker_update();
    ROS_INFO("Reached subgoal %d",++counter);
    if(counter==marker.points.size()-1){
      std::vector<geometry_msgs::Pose>().swap(goalQueue.poses);
      std::vector<geometry_msgs::Point>().swap(marker.points);
      std::vector<std_msgs::ColorRGBA>().swap(marker.colors);
      counter=0;
    }
  }
}

int main(int argc, char **argv){
  ros::init(argc,argv,"subgoal_publisher");
  ros::NodeHandle nh;
  ros::Subscriber subgoalSub=nh.subscribe("/move_base_simple/goal", 1000,goalCallback);
  ros::ServiceServer service=nh.advertiseService("/sendGoals", send);
  ros::Subscriber vehicle1Sub=nh.subscribe("/v1/filtered_ndt_current_pose", 1000, v1Callback);
  ros::Subscriber vehicle2Sub=nh.subscribe("/v2/filtered_ndt_current_pose", 1000, v2Callback);
  vis_pub = nh.advertise<visualization_msgs::Marker>( "/subgoalVisualization", 0 , true);
  goal_pub = nh.advertise<geometry_msgs::PoseArray>( "/subgoals", 0 , true);
  next_pub=nh.advertise<std_msgs::Bool>("/nextgoals",1,true);

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.ns = "goals";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  
  colour.a = 1.0; 
  colour.r = 0.3;
  colour.g = 0.8;
  colour.b = 0.3;
  marker.color=colour;
  while(nh.ok()){
    ros::spinOnce();
    if(marker.points.size())
      check_goals();
  }
}


