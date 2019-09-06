#ifndef SBS_AGENT_H
#define SBS_AGENT_H
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <queue>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <visualization_msgs/Marker.h>
#include "param.hpp"
#include <trajectory_tracker/TrajectoryPath.h>
#include <nav_msgs/Odometry.h>

class sbs_agent
{
 public:
  sbs_agent();
  void run();

 private:

  ros::NodeHandle nh;
  ros::Subscriber pathSub;
  ros::Subscriber subgoalSub;
  ros::Subscriber currentPoseSub;
  ros::Subscriber currentAmclPoseSub;
  ros::Subscriber gridSub;
  ros::Subscriber speedSub;
  ros::Subscriber yawSub;
  ros::Subscriber odomSub;
  ros::Subscriber partnerSpeedSub;
  ros::Subscriber partnerYawSub;
  ros::Subscriber partnerPoseSub;
  ros::Subscriber partnerAmclPoseSub;
  ros::Subscriber partnerTwistSub;
  ros::Subscriber partnerOdomSub;
  ros::Subscriber occupancyGridMapSubscriber;
  ros::Subscriber nextgoalSub;
  ros::Publisher vis_pub;
  ros::Publisher twist_pub;
  ros::Publisher gridMapPublisher;
  ros::Publisher trajectoryMarkPublisher;
  ros::Publisher trajectoryFilter1MarkPublisher;
  ros::Publisher trajectoryFilter2MarkPublisher;
  ros::Publisher trajectoryFilter3MarkPublisher;
  ros::Publisher optimalTrajectoryMarkPublisher;
  ros::Publisher optimalTrajectoryPublisher;
  ros::Publisher agentglobalpathPublisher;

  void pathCallback(const trajectory_tracker::TrajectoryPath::ConstPtr& input);
  void goalCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  void nextgoalCallback(const std_msgs::Bool::ConstPtr& msg);
  void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& input);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void speedCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void yawCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void partnerSpeedCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void partnerYawCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void partnerTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void partnerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void partnerAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void partnerOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void vizCallback(const visualization_msgs::Marker::ConstPtr& msg);
  void occupancyGridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input);
  
  bool generateTrajectories(std::vector< std::vector<double> > LaneList,
			    std::vector< std::vector< std::vector<double> > > trajectoryArrayCartesianList[3],std::vector< std::vector< std::vector<double> > > previousOptimalTrajectoryArrayCartesianList, double yaw, Eigen::Vector2d currentPos, double dv, double dw, Eigen::Vector2d &next, bool self);
  void publishOptimalTrajectoryToTrajectoryTracker( std::vector< std::vector< std::vector< double > > >& optimalTrajectoryArrayList );
  void copyTrajectoryArrayListToTrajectoryPathMessage( std::vector< std::vector< std::vector<double> > >& laneList, trajectory_tracker::TrajectoryPath& returnPathMessage);
  grid_map::Position applyTransform(grid_map::Position in, double x, double y, double yaw);
  Utility util;
  std::string partnerNs;

  std_msgs::Float64MultiArray self_LaneMessage;
  std::vector< std::vector<double> > self_LaneList;

  std_msgs::Float64MultiArray partner_LaneMessage;
  std::vector< std::vector<double> > partner_LaneList;

  //std::queue<Eigen::Vector2d> subgoals;
  //std::queue<Eigen::Vector2d> holdgoals;
  std::string frame_id;
  tf::Pose currentPose, partnerPose;
  grid_map::GridMap map;  
  grid_map::GridMap occmap;
  double dv, dw, pdv, pdw;
  double vehicle_length, vehicle_width;
  double delta_time=1.0;
  //double subgoal_distance_tolerance = 0.5;
  double hz=100;
  visualization_msgs::Marker marker;
  //int goalCounter=0;
  geometry_msgs::Twist stop;
  int numAgents,id;
  bool isVehicle, pIsVehicle;
  std::string ns;
  Eigen::Vector2d currentPos,partnerPos ;

  double social_distance;
  //std::string agentglobalpath_topic;
  bool right_side_path;
  bool use_collision;
  double distance_to_subgoal_ahead;
  double adjust_path_speed_constant;
  
  double timeMinSec;
  double timeMaxSec;
  double timeDeltaSec;

  double desiredSpeedPlus;
  double desiredSpeed;
  double desiredSpeedMinus;
  double desiredSpeedDelta; 

  double desiredLatPosition;
  double desiredLatPositionPlus;
  double desiredLatPositionMinus;
  double desiredLatPositionDelta;

  double roadMarginLeft;
  double roadMarginRight;
  double maxCentrifugalForce;
  double maxAcceleration;
  double maxCurvature;

  double polyTickTime;
  double arcLengthTickMeter;
};



#endif
