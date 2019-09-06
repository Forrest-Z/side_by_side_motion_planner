#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Vector3 v(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
  transform.setOrigin(v);
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_link")); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle n;

  ros::Subscriber currentPoseSub=n.subscribe("/filtered_ndt_current_pose",1,poseCallback);
  
  ros::spin();

  return 0;
}
