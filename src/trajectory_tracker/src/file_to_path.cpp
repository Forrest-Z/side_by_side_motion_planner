#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 
//#include <nav_msgs/Path.h>
#include <trajectory_tracker/TrajectoryPath.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "file_to_path");
  ros::NodeHandle n("~");

  std::string filename;
  n.param("filename", filename, std::string(""));
    
  if(filename.empty())
  {
    ROS_ERROR("filename parameter is emtpy");
    return -1;
  }



  FILE* file;
  if ((file = fopen(filename.c_str(), "r")) == NULL) 
  {
    ROS_ERROR("cannot open file %s", filename.c_str());
    return -1;
  }

  
  trajectory_tracker::TrajectoryPath  pose_vel_array;
  trajectory_tracker::TrajectoryMessage pose_vel;
		
  //nav_msgs::Path path_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "map";
  pose_vel.header = header;
  pose_vel_array.header = header;
  
  double x, y, z, th, vel, ang_vel, time;
  while(fscanf(file, "%lf %lf\n", &x, &y) >0)  
  // while(fscanf(file, "%lf, %lf %lf %lf\n", &time, &x, &y, &z) >0)  
  {
 
    //std::cerr << x << " " << y << " " << vel << "\n"; 
    //geometry_msgs::PoseStamped pose_stamped;
    pose_vel.header = header;
    pose_vel.pose.position.x = x;
    pose_vel.pose.position.y = y;
    //pose_vel.pose.position.z = z;
    pose_vel.pose.orientation.w = 1.0;

    //pose_vel.twist.linear.x = vel; // Linear Velocity
    //pose_vel.twist.angular.z = ang_vel;
    //pose_vel.header.frame_id = frameGlobal;
    //pose_vel.header.stamp = now;
    pose_vel.header.seq = pose_vel_array.poses.size();


    pose_vel_array.poses.push_back(pose_vel);
  }
  
  fclose(file);
  ros::Publisher pub = n.advertise<trajectory_tracker::TrajectoryPath>("/record_pose_vel_array", 1, true);
  

  pub.publish(pose_vel_array);

  ros::spin();

  return 0;
}
