#ifndef UTILITY_H
#define UTILITY_H
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include "sbs_motion/subgoalList.h"
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include "param.hpp"

class Utility
{
 public:

  Utility();
  void init(double delta_time,double relativeDistA, double relativeDistB, double relativeDistC, double relativeDistK,
		   double relativeAngleA, double relativeAngleB,double relativeAngleC,double relativeAngleK,
		   double relativeVelA, double relativeVelB,double relativeVelC,double relativeVelK,
		   double subgoalAngleA, double subgoalAngleB, double subgoalAngleC, double subgoalAngleK,
		   double obstacleDistA, double obstacleDistB, double obstacleDistK,
		   double velocityA, double velocityB, double velocityC, double velocityK,
		   double angularVelA, double angularVelB, double angularVelC, double angularVelK,
		   double accelA, double accelB, double accelC, double accelK);
  grid_map::GridMap unpartneredUtil(double dv, double dw, tf::Pose currentPose, grid_map::GridMap map, grid_map::Position next,tf::Pose partnerPose);
  grid_map::GridMap doubleUtil(grid_map::GridMap self, grid_map::GridMap partner,tf::Pose currentPose,tf::Pose partnerPose);
  double angle(double angle);
  grid_map::GridMap simplePartnered(double dv, double dw, tf::Pose currentPose, grid_map::GridMap map, grid_map::Position next, double pdv, double pdw, tf::Pose partnerPose);

  std::vector< std::vector< std::vector<double> > > trajectoryUtil(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn, 
								   std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut, 
								   Eigen::Vector2d next_subgoal, double yaw, std::vector<Eigen::Vector2d> objs, 
								   double& max, double& min, std::string ns);
  std::vector< std::vector< std::vector<double> > > combinedTrajectory(std::vector< std::vector< std::vector<double> > >& trajectoryArraySelf,
								       std::vector< std::vector< std::vector<double> > >& trajectoryArrayPartner,
								       std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut, 
								       double& max, double& min);

  std::vector< std::vector< std::vector<double> > > LuiscombinedTrajectory(std::vector< std::vector< std::vector<double> > >& trajectoryArraySelf,
								       std::vector< std::vector< std::vector<double> > >& trajectoryArrayPartner,
								       std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut, 
									   double& max, double& min,
									   Eigen::Vector2d selfPos, double selfYaw, Eigen::Vector2d partnerPos, double partnerYaw
									   );

  std::vector< std::vector< std::vector<double> > > trajectoryStop(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn,
								   std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut, 
								   Eigen::Vector2d pos, double yaw);
 private:
  inline double util(double x, double m, double s);
  inline double normAngle(double angle);
  inline double acuteAngle(double angle);
  double util_motion(grid_map::Position pos,double yaw,double dv,double delta_time,double angle, double vel);
  double util_env(grid_map::Position pos,double subgoalAngle,double angle, std::vector<Eigen::Vector2d> objs);
  double util_relative(grid_map::Position pos, grid_map::Position currentPos,double vel,  grid_map::Position currentPartner,grid_map::Position partner, double pYaw, double pdv);
  double Dist_Between_Points( double x1, double x2, double y1, double y2) ;

 
  Param relativeDist;
  Param relativeAngle;
  Param relativeVel;
  Param subgoalAngle;
  Param obstacleDist;
  Param velocity;
  Param angularVel;
  Param accel;
  double delta_time;


};

#endif
