#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <queue>
#include "sbs_motion/subgoalList.h"
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <math.h>
#include <Eigen/Dense>
#include "utility.hpp"
#include "sbs_agent.hpp"
#include <std_msgs/Bool.h>
#include "frene.hpp"
#include "trajectory_array.h"
#include "lane_list.h"
#include <trajectory_tracker/TrajectoryPath.h>
#include <nav_msgs/Odometry.h>

#define TURN_MAX 1.6
#define SPEED_MAX 1.5
//#define DISTANCE_TOLERANCE 1.0

void drawTrajectory( std::vector< std::vector< std::vector<double> > >& trajectoryList, ros::Publisher& trajectoryMarkPublisher,
		     double rColor, double gColor, double bColor, int skipCount, int type , double max, double min, bool opt);
std_msgs::ColorRGBA parulaColorMap( double value );

void sbs_agent::partnerPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tf::Vector3 v(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  partnerPose.setOrigin(v);
  partnerPose.setRotation(q);
  partnerPos[0]=msg->pose.position.x;
  partnerPos[1]=msg->pose.position.y;
}


/*
void sbs_agent::partnerSpeedCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  pdv=msg->vector.x;
  if (pdv<1){
    pdv=1;
  }
}

void sbs_agent::partnerYawCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  pdw=msg->vector.x;

}
*/
void sbs_agent::partnerOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  pdv=msg->twist.twist.linear.x;
  pdw=msg->twist.twist.angular.z;
  if (pdv<1){
    pdv=1;
  }
}


void sbs_agent::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tf::Vector3 v(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  currentPose.setOrigin(v);
  currentPose.setRotation(q);
  currentPos[0]=msg->pose.position.x;
  currentPos[1]=msg->pose.position.y;

}


void sbs_agent::speedCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  dv=msg->vector.x;
    if (dv<1){
    dv=1;
  }
}

void sbs_agent::yawCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  dw=msg->vector.x;
}

void sbs_agent::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  dv=msg->twist.twist.linear.x;
  dw=msg->twist.twist.angular.z;
  if (dv<1){
    dv=1;
  }
}



void sbs_agent::goalCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  int size=msg->poses.size();
  ROS_INFO("%s: Received %d subgoals",ns.c_str(),size);
  for(int i=0; i<size;i++){
    Eigen::Vector2d goal(msg->poses[i].position.x,msg->poses[i].position.y);
    ROS_INFO("%s: X: %f \t Y: %f",ns.c_str(),goal.x(),goal.y());
    holdgoals.push(goal);
  }

}

void sbs_agent::occupancyGridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input)
{
  grid_map::GridMapRosConverter::fromOccupancyGrid(*input, "traversability", map);
}

void sbs_agent::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& input)
{  
  grid_map::GridMap tmp;
  grid_map::GridMapRosConverter::fromMessage(*input,  tmp);
  occmap=tmp;
}


sbs_agent::sbs_agent():
  dv(0),
  dw(0),
  pdv(0),
  pdw(0)
{

  nh=ros::NodeHandle();
  ros::NodeHandle np("~");
  std::string partnerNs;

  np.getParam("partner",partnerNs);
  np.param<int>("id",id,1);
  np.param<int>("agents",numAgents,1);
  np.param<double>( "hz", hz, 100 );
  np.param<double>( "delta_time", delta_time, 1.0 );

  np.param<double>( "initialPoseX", currentPos(0), 0 );
  np.param<double>( "initialPoseY", currentPos(1), 0 );
 
  np.param<double>( "length", vehicle_width, 1 );
  np.param<double>( "width", vehicle_length, 1 );

  np.param<double>( "subgoal_distance_tolerance", subgoal_distance_tolerance, 0.5 );
  
 
  //Frene parameters
  np.param<double>( "timeMinSec", timeMinSec, 1.0 );
  np.param<double>( "timeMaxSec", timeMaxSec, 1.5);
  np.param<double>( "timeDeltaSec", timeDeltaSec, 0.5 );
  
  np.param<double>( "desiredSpeed", desiredSpeed, 1.0 );
  np.param<double>( "desiredSpeedPlus", desiredSpeedPlus, 0.75 );
  np.param<double>( "desiredSpeedMinus", desiredSpeedMinus, -1.0 );
  np.param<double>( "desiredSpeedDelta", desiredSpeedDelta, 0.75 );

  np.param<double>( "desiredLatPosition", desiredLatPosition, 0.0 );
  np.param<double>( "desiredLatPositionPlus", desiredLatPositionPlus, 0.6 );
  np.param<double>( "desiredLatPositionMinus", desiredLatPositionMinus, -0.6 );
  np.param<double>( "desiredLatPositionDelta", desiredLatPositionDelta, 0.3 );

  np.param<double>( "arcLengthTickMeter", arcLengthTickMeter, 0.4 );
  np.param<double>( "roadMarginLeft", roadMarginLeft, 0.7 );
  np.param<double>( "roadMarginRight", roadMarginRight, 0.7 );
  np.param<double>( "maxCentrifugalForce", maxCentrifugalForce, 1.5 );
  np.param<double>( "maxAcceleration", maxAcceleration, 1.0 );
  np.param<double>( "maxCurvature", maxCurvature, 17.5 );

  np.param<double>( "polyTickTime", polyTickTime, 0.1 );
  
  //Utility Parameters
  double relativeDistA,relativeDistB,relativeDistC,relativeDistK,
    relativeAngleA,relativeAngleB,relativeAngleC,relativeAngleK,
    relativeVelA,relativeVelB,relativeVelC,relativeVelK,
    subgoalAngleA,subgoalAngleB,subgoalAngleC,subgoalAngleK,
    obstacleDistA,obstacleDistB,obstacleDistK,
    velocityA,velocityB,velocityC,velocityK,
    angularVelA,angularVelB,angularVelC,angularVelK,
    accelA,accelB,accelC,accelK;

  np.param<double>( "relativeDistA", relativeDistA, 0.25 );
  np.param<double>( "relativeDistB", relativeDistB, 2.00 );
  np.param<double>( "relativeDistC", relativeDistC, 1.00 );
  np.param<double>( "relativeDistK", relativeDistK, 0.25 );

  np.param<double>( "relativeAngleA", relativeAngleA, 0.08 );
  np.param<double>( "relativeAngleB", relativeAngleB, 3.00 );
  np.param<double>( "relativeAngleC", relativeAngleC, M_PI/2 );
  np.param<double>( "relativeAngleK", relativeAngleK, 0.35 );

  np.param<double>( "relativeVelA", relativeVelA, 0.20 );
  np.param<double>( "relativeVelB", relativeVelB, 1.20 );
  np.param<double>( "relativeVelC", relativeVelC, 0.00 );
  np.param<double>( "relativeVelK", relativeVelK, 0.05 );

  np.param<double>( "subgoalAngleA", subgoalAngleA, 0.45 );
  np.param<double>( "subgoalAngleB", subgoalAngleB, 1.00 );
  np.param<double>( "subgoalAngleC", subgoalAngleC, 0.00 );
  np.param<double>( "subgoalAngleK", subgoalAngleK, 0.30 );
  
  np.param<double>( "obstacleDistA", obstacleDistA, 20.0 );
  np.param<double>( "obstacleDistB", obstacleDistB, 0.40 );
  np.param<double>( "obstacleDistK", obstacleDistK, 0.03 );

  np.param<double>( "velocityA", velocityA, 0.30 );
  np.param<double>( "velocityB", velocityB, 1.60 );
  np.param<double>( "velocityC", velocityC, 1.10 );
  np.param<double>( "velocityK", velocityK, 0.05 );

  np.param<double>( "angularVelA", angularVelA, 0.70 );
  np.param<double>( "angularVelB", angularVelB, 1.00 );
  np.param<double>( "angularVelC", angularVelC, 0.00 );
  np.param<double>( "angularVelK", angularVelK, 0.01 );

  np.param<double>( "accelA", accelA, 0.20 );
  np.param<double>( "accelB", accelB, 1.00 );
  np.param<double>( "accelC", accelC, 0.00 );
  np.param<double>( "accelK", accelK, 0.01 );


  //ns = ros::this_node::getNamespace();
  //std::cerr << "\n "  << ros::this_node::getNamespace() << " " << ns.c_str();



  ns = ros::this_node::getNamespace().substr(2);
  ROS_INFO("%s: Initializing agent",ns.c_str());


  //std::cerr << "\n agent: " << ns.c_str() << "\n\n" ;

  subgoalSub=nh.subscribe("/subgoals", 1000, &sbs_agent::goalCallback,this);
  currentPoseSub=nh.subscribe("filtered_ndt_current_pose",1,&sbs_agent::poseCallback,this);

  gridSub=nh.subscribe("occmap",1,&sbs_agent::gridMapCallback,this);
  // speedSub=nh.subscribe("vehicle_speed",1,&sbs_agent::speedCallback,this); 
  // yawSub=nh.subscribe("vehicle_yawrate",1,&sbs_agent::yawCallback,this);
  
  odomSub=nh.subscribe("odom",1,&sbs_agent::odomCallback,this);
  
  occupancyGridMapSubscriber=nh.subscribe("map",1,&sbs_agent::occupancyGridMapCallback,this);
  twist_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/agent",1000);

  trajectoryMarkPublisher = nh.advertise<visualization_msgs::Marker>("trajectory_mark", 10);
  trajectoryFilter1MarkPublisher = nh.advertise<visualization_msgs::Marker>("trajectory_mark_filter1", 10);
  trajectoryFilter2MarkPublisher = nh.advertise<visualization_msgs::Marker>("trajectory_mark_filter2", 10);
  trajectoryFilter3MarkPublisher = nh.advertise<visualization_msgs::Marker>("trajectory_mark_filter3", 10);
  optimalTrajectoryMarkPublisher = nh.advertise<visualization_msgs::Marker>("trajectory_mark_optimal", 10);
  optimalTrajectoryPublisher     = nh.advertise<trajectory_tracker::TrajectoryPath>("trajectory_tracker/path", 10); 

  //Subscribe to partner pose and twist
  if(numAgents!=1){
    partnerPoseSub=nh.subscribe(partnerNs+"/filtered_ndt_current_pose",1,&sbs_agent::partnerPoseCallback,this);
    // partnerSpeedSub=nh.subscribe(partnerNs+"/vehicle_speed",1,&sbs_agent::partnerSpeedCallback,this);
    // partnerYawSub=nh.subscribe(partnerNs+"/vehicle_yawrate",1,&sbs_agent::partnerYawCallback,this);
    partnerOdomSub=nh.subscribe(partnerNs+"/odom",1,&sbs_agent::partnerOdomCallback,this);
  
    partnerPos<<0,0;

  }
  gridMapPublisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  
  isVehicle=true;
  pIsVehicle=true;
  
  map.setFrameId("/map");
  map.setGeometry(grid_map::Length(200,200),0.2,  grid_map::Position(0,0));
  
  
  occmap.add("traversability",0.0);
  occmap.setGeometry(grid_map::Length(5,5),0.2,  grid_map::Position(0,0));

  stop.linear.x = stop.linear.y = stop.linear.z = 0;
  stop.angular.x = stop.angular.y = stop.angular.z = 0;

  ROS_INFO("%s: Initializing Utility System",ns.c_str());
  util.init(delta_time,relativeDistA,relativeDistB,relativeDistC,relativeDistK,
    relativeAngleA,relativeAngleB,relativeAngleC,relativeAngleK,
    relativeVelA,relativeVelB,relativeVelC,relativeVelK,
    subgoalAngleA,subgoalAngleB,subgoalAngleC,subgoalAngleK,
    obstacleDistA,obstacleDistB,obstacleDistK,
    velocityA,velocityB,velocityC,velocityK,
    angularVelA,angularVelB,angularVelC,angularVelK,
    accelA,accelB,accelC,accelK);

  
}

// Returns trajectoryArrayCartesianList[1]   after collision filtering
bool sbs_agent::generateTrajectories(std::vector< std::vector< std::vector<double> > > trajectoryArrayCartesianList[3], std::vector< std::vector< std::vector<double> > > previousOptimalTrajectoryArrayCartesianList, double yaw, Eigen::Vector2d currentPos, double dv, double dw, Eigen::Vector2d next, bool self)
{
  std::vector< std::vector<double> > projectedPath;
  double pointX=currentPos.x();
  double pointY=currentPos.y();
  double theta=yaw;
  Eigen::Vector2d toNext=(next-currentPos);
  /*If the next subgoal is less than 1 meter away and there is another subgoal after it, give a path that first goes to the next subgoal and then from the next subgoal to the subgoal after it
  Otherwise, give a path that is just 3 meters in the direction of the next subgoal
  */
  if(toNext.norm()<1&&!subgoals.empty()){
    double deltaX=toNext.x()/3;
    double deltaY=toNext.y()/3;
    
    for( int i=0; i<3;i++){
      std::vector<double> point;
      pointX+=deltaX*i;
      pointY+=deltaY*i;
      point.push_back(pointX);
      point.push_back(pointY);
      projectedPath.push_back(point);
    }
    pointX=next.x();
    pointY=next.y();
    Eigen::Vector2d nextGoal=subgoals.front();
    toNext=nextGoal-next;
  }
  toNext.normalize();
  double deltaX=toNext.x()/3;
  double deltaY=toNext.y()/3;
  
  for( int i=0; i<=9;i++){
    std::vector<double> point;
    pointX+=deltaX*i;
    pointY+=deltaY*i;
    point.push_back(pointX);
    point.push_back(pointY);
    projectedPath.push_back(point);
  }
  
  //init frene frame
  std::vector< std::vector<double> > xyResamplingList;
  tk::spline _splineX;
  tk::spline _splineY;
  Frene frene(projectedPath, 0.3, xyResamplingList, _splineX, _splineY, ns, self);
  
  FreneState initialState;
  FreneState initialStateFromSensor;  
  FreneState initialStateFromPreviousTrajectory;  

  //generate lateral & longitudinal trajectory
  std::vector< std::vector< std::vector<double> > > trajectoryArrayFreneList;
  std::vector< std::vector< std::vector<double> > > paramArrayList;
  
  if( self && previousOptimalTrajectoryArrayCartesianList.size() > 0 && previousOptimalTrajectoryArrayCartesianList[0].size() > 0){
    double xCurrent = currentPos.x();
    double yCurrent = currentPos.y();
    double vxCurrent = dv;
    frene.computeInitialStateDirectFromPreviousTrajectoryWithSplineResampling( xCurrent, yCurrent, 
									       previousOptimalTrajectoryArrayCartesianList, 
									       initialStateFromPreviousTrajectory,
									       0.4
									       );           
    initialState = initialStateFromPreviousTrajectory;
    initialState.s = initialStateFromSensor.s;
    initialState.d = initialStateFromSensor.d;  
    
  }else{
    frene.convertFromCartesianState(0,currentPos.x(),currentPos.y(),yaw,dv,0,dw/dv,dw,initialStateFromSensor,arcLengthTickMeter, _splineX, _splineY, ns.c_str(), self );  
    initialState = initialStateFromSensor;
  }
  
  frene.generateLateralAndLongitudinalTrajectoryVelocityKeeping( initialState, 
  								 trajectoryArrayFreneList,
  								 paramArrayList,
  								 timeMinSec, timeMaxSec, timeDeltaSec, 
  								 desiredSpeed, desiredSpeedMinus, desiredSpeedPlus, desiredSpeedDelta, 
  								 desiredLatPosition, desiredLatPositionMinus,  desiredLatPositionPlus,  desiredLatPositionDelta, polyTickTime );

  
  frene.convertMergedTrajectoryArrayToCartesian( trajectoryArrayFreneList, trajectoryArrayCartesianList[0], _splineX, _splineY );
  
 
  
  if(occmap["traversability"].any())
    frene.filterByCollision(trajectoryArrayCartesianList[0], trajectoryArrayCartesianList[1],occmap,vehicle_length, vehicle_width,currentPos,yaw);
  else
    trajectoryArrayCartesianList[1]=trajectoryArrayCartesianList[0];

  return trajectoryArrayCartesianList[0].size()!=0;
}

//Copy frenet planner optimaltrajectory to trajectory_tracker path message
void sbs_agent::copyTrajectoryArrayListToTrajectoryPathMessage( std::vector< std::vector< std::vector<double> > >& laneList, trajectory_tracker::TrajectoryPath& returnPathMessage)
{
  //input data
  ros::Time currentTime = ros::Time::now();
  
  returnPathMessage.poses.clear();

  returnPathMessage.header.stamp = currentTime;
  returnPathMessage.header.frame_id = "map";
      
  int seqCount = 0;

  static double prevTheta = 0.0;
  static double thetaWrapped = 0.0;

  for( std::vector< std::vector< std::vector<double> > >::iterator itr0=laneList.begin(); itr0 != laneList.end(); itr0++ ){            
    
    //for mpc tracker
    double xprev = 0;
    double yprev = 0;
    bool isFirstFlag = true;
    double uprev = 0;
    
    for( std::vector< std::vector<double> >::iterator itr1=itr0->begin(); itr1 != itr0->end(); itr1++ ){                
        
      trajectory_tracker::TrajectoryMessage message;
      
      double x = (*itr1)[0];
      double y = (*itr1)[1];
      double t = (*itr1)[2];
      double theta = (*itr1)[3];    
      double dtTheta = (*itr1)[4];    
      double vx = (*itr1)[5];
      double ax = (*itr1)[6];
      double kx = (*itr1)[7];
      
      //for mpc tracker
      if(isFirstFlag){
	xprev = x;
	yprev = y;
	isFirstFlag = false;
      }
      double diff = sqrt( pow(x - xprev, 2.0) + pow(y-yprev, 2.0));
      double u =  diff + uprev;
      xprev = x;
      yprev = y;
      uprev = u;      
      double dt = theta - prevTheta;
      while (dt < -M_PI){ dt += 2.0 * M_PI;}
      while (dt > M_PI){ dt -= 2.0 * M_PI;}
      thetaWrapped += dt;
      prevTheta = theta;
      
      //message.header.stamp = currentTime + ros::Duration(t);
      message.header.stamp =  currentTime + ros::Duration(t);
      message.header.seq = seqCount;
      message.header.frame_id = "map";
      
      //some values are for mpc tracker
      message.pose.position.x = x;
      message.pose.position.y = y;
      message.pose.position.z = t;
      message.pose.orientation.x = 0;//theta;
      message.pose.orientation.y = 0;//thetaWrapped;
      message.pose.orientation.z = 0;//u;
      message.pose.orientation.w = 1;//sqrt(pow(theta,2.0)+pow(thetaWrapped,2.0)+pow(u,2.0));
      
      //message.twist.linear.x = vx;
      std::vector<double> lastElement = (*itr0).back();
      //message.twist.linear.x = lastElement[5];//send final speed      
      message.twist.linear.x = vx;
      message.twist.linear.y = ax;
      message.twist.linear.z = kx;
      message.twist.angular.x = thetaWrapped;
      message.twist.angular.y = u;
      message.twist.angular.z = dtTheta*kx;
      
      returnPathMessage.poses.push_back(message);
      
      seqCount++;
    }
  }

}



//publish to trajectory tracker
void sbs_agent::publishOptimalTrajectoryToTrajectoryTracker( std::vector< std::vector< std::vector< double > > >& optimalTrajectoryArrayList )
{  
  trajectory_tracker::TrajectoryPath trajectoryPathMessage;
  copyTrajectoryArrayListToTrajectoryPathMessage( optimalTrajectoryArrayList, trajectoryPathMessage);  
  
  optimalTrajectoryPublisher.publish(trajectoryPathMessage);    
}

grid_map::Position sbs_agent::applyTransform(grid_map::Position in, double x, double y, double yaw){
  Eigen::Rotation2Dd rot(yaw);
  Eigen::Vector2d trans(x,y);
  return rot*in+trans;
}

void sbs_agent::run(){
  ros::Rate r(hz);
  Eigen::Vector2d last;
  while(nh.ok()){

    if(subgoals.empty()){
      last=currentPos;
      std::swap(subgoals,holdgoals);
      goalCounter=0;
      twist_pub.publish(stop);
      ros::spinOnce();
      r.sleep();
      continue;
    }
    Eigen::Vector2d goal=subgoals.front();
    subgoals.pop();
    ROS_WARN("%s: travelling to subgoal %d", ns.c_str(),++goalCounter);

    Eigen::Vector2d prevToGoal=(goal-last).normalized();
    Eigen::Rotation2Dd rot(M_PI/2);
    Eigen::Vector2d orth=rot*prevToGoal;

   
    
    Eigen::Vector2d next=goal+0.5*(2*id-numAgents-1)*orth.normalized();
    Eigen::Vector2d pNext=goal-0.5*(2*id-numAgents-1)*orth.normalized();
    last=goal;

    //While not at next
    while((next-currentPos).norm() >  subgoal_distance_tolerance && (goal-currentPos).norm()> subgoal_distance_tolerance  &&
	  (pNext-partnerPos).norm() > subgoal_distance_tolerance && (goal-partnerPos).norm()> subgoal_distance_tolerance ){
      ros::spinOnce();


      double yaw=util.angle(tf::getYaw(currentPose.getRotation()));

      //Create vector of intraversible locations
      std::vector<Eigen::Vector2d> objs;
      bool success=false;
      
      // if(occmap["traversability"].any()){
      // 	grid_map::Matrix& data = occmap["traversability"];
      // 	for (grid_map::GridMapIterator iterator(occmap);!iterator.isPastEnd(); ++iterator) {
      // 	  if(occmap.at("traversability",*iterator)!=0){
      // 	    grid_map::Position pos;
      // 	    occmap.getPosition(*iterator,pos);
      // 	    objs.push_back(applyTransform(pos, currentPos.x(),currentPos.y(), yaw));
      // 	  }
      // 	}
      // }
      
      grid_map::GridMap sub=map.getSubmap(currentPos, grid_map::Length(4.0,4.0), success);  
      if(sub["traversability"].any()){
	grid_map::Matrix& data = sub["traversability"];
	for (grid_map::GridMapIterator iterator(sub);!iterator.isPastEnd(); ++iterator) {
	  if(sub.at("traversability",*iterator)!=0){
	    grid_map::Position pos;
	    sub.getPosition(*iterator,pos);
	    objs.push_back(pos);
	  }
	}
      }
      

      static std::vector< std::vector< std::vector<double> > > previousOptimalTrajectoryArrayCartesianList;
      std::vector< std::vector< std::vector<double> > > trajectoryArrayCartesianList[5];
      double max, min;
      //Try to generate trajectories for self
      if(!generateTrajectories(trajectoryArrayCartesianList, previousOptimalTrajectoryArrayCartesianList, yaw, currentPos, dv, dw, next,true)){
	ROS_ERROR("%s: Error generating self trajectories",ns.c_str());
	twist_pub.publish(stop);
	continue;
      }

      
      
      //check for traversability, if none, choose a trajectory with 0 speed and continue
      if( trajectoryArrayCartesianList[1].size() == 0
	  //&& trajectoryArrayCartesianList[0].size()==0
	  //&& trajectoryArrayCartesianList[2].size()==0 && trajectoryArrayCartesianList[3].size()==0 // && trajectoryArrayCartesianList[4].size()==0 ){
	  ){
	ROS_ERROR("%s No safe self trajectories..",ns.c_str());
	twist_pub.publish(stop);
	continue;
      }
      

      std::vector< std::vector< std::vector<double> > > optimalTrajectoryArrayCartesianList;
      optimalTrajectoryArrayCartesianList=util.trajectoryUtil(trajectoryArrayCartesianList[1],trajectoryArrayCartesianList[3],next,yaw,objs,max,min);

      //forPartner
      if(numAgents>1){
	double pMax, pMin;
	double pYaw=util.angle(tf::getYaw(partnerPose.getRotation()));
	std::vector< std::vector< std::vector<double> > > pTrajectoryArrayCartesianList[5];
	if(generateTrajectories(pTrajectoryArrayCartesianList, previousOptimalTrajectoryArrayCartesianList, pYaw, partnerPos, pdv, pdw,pNext,false)){
	  if(pTrajectoryArrayCartesianList[1].size()==0)
	    pTrajectoryArrayCartesianList[4]=util.trajectoryStop(trajectoryArrayCartesianList[3],trajectoryArrayCartesianList[4],partnerPos,pYaw);
	  else {
	    util.trajectoryUtil(pTrajectoryArrayCartesianList[0],pTrajectoryArrayCartesianList[3],pNext,pYaw,objs,pMax,pMin);
	    pTrajectoryArrayCartesianList[4]=util.combinedTrajectory(trajectoryArrayCartesianList[3],pTrajectoryArrayCartesianList[3],trajectoryArrayCartesianList[4],pMax,pMin);     
	  }
	  TrajectoryArray::copyTrajectoryArrayListToTrajectoryArrayList(pTrajectoryArrayCartesianList[4],optimalTrajectoryArrayCartesianList);
	  TrajectoryArray::copyTrajectoryArrayListToTrajectoryArrayList(trajectoryArrayCartesianList[4],trajectoryArrayCartesianList[3]);
	}
	else
	  ROS_ERROR("%s: Error generating partner trajectories",ns.c_str());
      }

      drawTrajectory( trajectoryArrayCartesianList[3], trajectoryFilter2MarkPublisher, 0.5, 0.5, 0.0, 0, visualization_msgs::Marker::LINE_LIST,max,min,false );//trajectories
      drawTrajectory( optimalTrajectoryArrayCartesianList, optimalTrajectoryMarkPublisher, 0.5, 0.5, 0.0, 0, visualization_msgs::Marker::LINE_LIST, 0,min,true);//optimal

      TrajectoryArray::copyTrajectoryArrayListToTrajectoryArrayList(optimalTrajectoryArrayCartesianList,previousOptimalTrajectoryArrayCartesianList);
      publishOptimalTrajectoryToTrajectoryTracker( optimalTrajectoryArrayCartesianList );
      r.sleep();
      
    }

    ROS_WARN("%s: Arrived at goal %d",ns.c_str(),goalCounter);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbs_agent");
  sbs_agent sbs;
  sbs.run();
  return 0;
}







//For displaying trajectories in rviz
void drawTrajectory( std::vector< std::vector< std::vector<double> > >& trajectoryList, ros::Publisher& trajectoryMarkPublisher,
		     double rColor, double gColor, double bColor, int skipCount, int type , double max=1.0, double min=0.0, bool opt=false)
{
  visualization_msgs:: Marker lineStrip;  
  lineStrip.points.clear();

  static bool first=true;
  lineStrip.header.frame_id = "map";
  lineStrip.type = type;
  lineStrip.scale.x = 0.02;
  if(opt)
    lineStrip.scale.x = 0.05;    
  lineStrip.color.r = rColor;
  lineStrip.color.g = gColor;
  lineStrip.color.b = bColor;
  lineStrip.color.a = 1.0;   
  //start compute trajectory
  static int count = 1;
  for( std::vector< std::vector< std::vector<double> > >::iterator trajectory=trajectoryList.begin(); trajectory != trajectoryList.end(); trajectory++ ){        
    if( count > skipCount ){
      //draw trajectory
      
 
      geometry_msgs::Point point;
      std_msgs::ColorRGBA each_color;
      
      int numOfPoints = 0;     
      double util=(*trajectory).back().back()-min;
      double vxmax = max-min;
      double gamma = (util)/vxmax;
      std_msgs::ColorRGBA parula = parulaColorMap( gamma );

      for( std::vector< std::vector<double> >::iterator it = trajectory->begin(); it != trajectory->end(); ++it){    
    	point.x = (*it)[0];
	point.y = (*it)[1];
	point.z = (*it)[21]-min;//kx*10;	
	

	if(!opt){
	  each_color = parula;
	}else{
	  each_color.r=1.0;
	  each_color.g=0;
	  each_color.b=0;
	  each_color.a=1.0;
	    }
	lineStrip.points.push_back(point);   
	lineStrip.colors.push_back(each_color);

	if(numOfPoints == 0){	  
	}else{
	  lineStrip.points.push_back(point);   
	  lineStrip.colors.push_back(each_color);
	  numOfPoints++;
	}	
	numOfPoints++;
      }
      if( fabs(numOfPoints/2.0) > 1e-04 ){
	lineStrip.points.push_back(point); 	
	lineStrip.colors.push_back(each_color);
      }
      numOfPoints = 0;
            
      count = 1;
    }
    count++;
  }
  //  std::cout << "count " << count << std::endl;
  count = 1;
  trajectoryMarkPublisher.publish(lineStrip);
  first=false;
}


std_msgs::ColorRGBA parulaColorMap( double value )
{
  double parulaMap[][3]=
    {//64 rows
      {0.2081, 0.1663, 0.5292},
      {0.2116, 0.1898, 0.5777},		
      {0.2123, 0.2138, 0.6270},
      {0.2081, 0.2386, 0.6771},
      {0.1959, 0.2645, 0.7279},
      {0.1707, 0.2919, 0.7792},
      {0.1253, 0.3242, 0.8303},
      {0.0591, 0.3598, 0.8683},
      {0.0117, 0.3875, 0.8820},
      {0.0060, 0.4086, 0.8828},
      {0.0165, 0.4266, 0.8786},
      {0.0329, 0.4430, 0.8720},
      {0.0498, 0.4586, 0.8641},
      {0.0629, 0.4737, 0.8554},
      {0.0723, 0.4887, 0.8467},
      {0.0779, 0.5040, 0.8384},
      {0.0793, 0.5200, 0.8312},
      {0.0749, 0.5375, 0.8263},
      {0.0641, 0.5570, 0.8240},
      {0.0488, 0.5772, 0.8228},
      {0.0343, 0.5966, 0.8199},
      {0.0265, 0.6137, 0.8135},
      {0.0239, 0.6287, 0.8038},
      {0.0231, 0.6418, 0.7913},
      {0.0228, 0.6535, 0.7768},
      {0.0267, 0.6642, 0.7607},
      {0.0384, 0.6743, 0.7436},
      {0.0590, 0.6838, 0.7254},
      {0.0843, 0.6928, 0.7062},
      {0.1133, 0.7015, 0.6859},
      {0.1453, 0.7098, 0.6646},
      {0.1801, 0.7177, 0.6424},
      {0.2178, 0.7250, 0.6193},
      {0.2586, 0.7317, 0.5954},
      {0.3022, 0.7376, 0.5712},
      {0.3482, 0.7424, 0.5473},
      {0.3953, 0.7459, 0.5244},
      {0.4420, 0.7481, 0.5033},
      {0.4871, 0.7491, 0.4840},
      {0.5300, 0.7491, 0.4661},
      {0.5709, 0.7485, 0.4494},
      {0.6099, 0.7473, 0.4337},
      {0.6473, 0.7456, 0.4188},
      {0.6834, 0.7435, 0.4044},
      {0.7184, 0.7411, 0.3905},
      {0.7525, 0.7384, 0.3768},
      {0.7858, 0.7356, 0.3633},
      {0.8185, 0.7327, 0.3498},
      {0.8507, 0.7299, 0.3360},
      {0.8824, 0.7274, 0.3217},
      {0.9139, 0.7258, 0.3063},
      {0.9450, 0.7261, 0.2886},
      {0.9739, 0.7314, 0.2666},
      {0.9938, 0.7455, 0.2403},
      {0.9990, 0.7653, 0.2164},
      {0.9955, 0.7861, 0.1967},
      {0.9880, 0.8066, 0.1794},
      {0.9789, 0.8271, 0.1633},
      {0.9697, 0.8481, 0.1475},
      {0.9626, 0.8705, 0.1309},
      {0.9589, 0.8949, 0.1132},
      {0.9598, 0.9218, 0.0948},
      {0.9661, 0.9514, 0.0755},
      {0.9763, 0.9831, 0.0538}
    };

  std_msgs::ColorRGBA ret;
  
  int num=0;
  num = (int)(floor((64-1)*value));//value: 0-1
  
  if( num < 0 ){ num = 0; }
  if( num >= 64 ){ num = 63; }
    
  ret.r = parulaMap[num][0];
  ret.g = parulaMap[num][1];  
  ret.b = parulaMap[num][2];
  ret.a = 1;
  
  return ret;
}
