#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <math.h>
#include <Eigen/Dense>
#include "utility.hpp"
#include "param.hpp"
#include "traversable.hpp"
#define LENGTH 2.0
Utility::Utility(){}

void Utility::init(double delta_time, 
		   double relativeDistA, double relativeDistB, double relativeDistC, double relativeDistK,
		   double relativeAngleA, double relativeAngleB,double relativeAngleC,double relativeAngleK,
		   double relativeVelA, double relativeVelB,double relativeVelC,double relativeVelK,
		   double subgoalAngleA, double subgoalAngleB, double subgoalAngleC, double subgoalAngleK,
		   double obstacleDistA, double obstacleDistB, double obstacleDistK,
		   double velocityA, double velocityB, double velocityC, double velocityK,
		   double angularVelA, double angularVelB, double angularVelC, double angularVelK,
		   double accelA, double accelB, double accelC, double accelK)
{
  this->delta_time=delta_time;
  relativeDist = Param(relativeDistA, relativeDistB, relativeDistC, relativeDistK); //k=0.32
  relativeAngle= Param(relativeAngleA,relativeAngleB,relativeAngleC,relativeAngleK); //k=0.42
  relativeVel  = Param(relativeVelA,relativeVelB,relativeVelC,relativeVelK); //k=0.05
  subgoalAngle = Param(subgoalAngleA,subgoalAngleB,subgoalAngleC,subgoalAngleK); 
  obstacleDist = Param(obstacleDistA,obstacleDistB,obstacleDistK); //k=0.03
  velocity     = Param(velocityA,velocityB,velocityC,velocityK);//k=0.05
  angularVel   = Param(angularVelA,angularVelB,angularVelC,angularVelK);//k=0.01
  accel        = Param(accelA,accelB,accelC,accelK);//k=0.01
}

double Utility::angle(double angle){
  return normAngle(angle);
}

double Utility::Dist_Between_Points( double x1, double y1, double x2, double y2){
  return ( sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) ) );
}

//double Utility::Dist_Between_Vector_Points( double x1, double x2, double y1, double y2){
//  return ( sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) ) );
//}

double Utility::util_motion(grid_map::Position pos,double yaw,double dv,double delta_time,double angle, double vel){
  double utility=0;
  //Velocity
  utility+=velocity.util(vel);

  //Angular Velocity
  utility+=angularVel.util(normAngle(angle-yaw)/(delta_time));
  
  //Acceleration
  double accel=(vel-dv)/delta_time;
  utility += this->accel.util(accel);

  return utility;
}

double Utility::util_env(grid_map::Position pos,double subgoalAngle,double angle,std::vector<Eigen::Vector2d> objs){
  double utility=0;
  //Moving towards Subgoal 
  utility+=this->subgoalAngle.util(subgoalAngle-angle);

  //Distance to obstacle
  if( !objs.empty()){
    double obstDist=DBL_MAX;
    for(std::vector<Eigen::Vector2d>::iterator itr=objs.begin(); itr!=objs.end(); itr++)
      if((pos-*itr).norm()<obstDist)
	obstDist=(pos-*itr).norm();
    utility+=obstacleDist.util(obstDist);
  }
  return utility;
}

double Utility::util_relative(grid_map::Position pos, grid_map::Position currentPos,double vel, grid_map::Position partnerPos, grid_map::Position pPos, double pYaw, double pdv)
{
  double utility=0;
  Eigen::Vector2d relativeDist=pos-pPos;
  //relative Distance
  utility+=this->relativeDist.util(relativeDist.norm());

 //relative Vel 
  utility+=relativeVel.util(abs(vel-pdv));

  //relative Angle
  //Calculate partner's angle to travel to that cell
  Eigen::Vector2d partnerDisp=pPos-partnerPos;
  double anglePartner=atan2(partnerDisp.y(),partnerDisp.x());
  //Angle of the vector between predicted positions
  double angleBtwn=atan2(relativeDist.y(),relativeDist.x());
  double relAngle=normAngle(anglePartner-angleBtwn);
  utility+=relativeAngle.util(relAngle);

  return utility;
}


inline double Utility::normAngle(double angle){
  while(angle>=M_PI)
    angle-=2*M_PI;
  while(angle<=-M_PI)
    angle+=2*M_PI;
  return angle;
}

inline double Utility::acuteAngle(double angle){
  while(angle>=M_PI)
    angle-=M_PI;
  while(angle<=0)
    angle+=M_PI;
  return angle;
}


/*
grid_map::GridMap Utility::unpartneredUtil(double dv, double dw, tf::Pose currentPose, grid_map::GridMap map, grid_map::Position next,tf::Pose partnerPose)
{
  double max = -DBL_MAX;;
  tf::Vector3 center = currentPose.getOrigin();
  double yaw = tf::getYaw(currentPose.getRotation());  
  double theta = normAngle((dw * delta_time)+yaw);
  double x,y;
  x = dv * cos(theta) * delta_time + center.getX();
  y = dv * sin(theta) * delta_time + center.getY();

  grid_map::Position currentPos=grid_map::Position(center.getX(),center.getY());
  grid_map::Position extrapPos=currentPos;
  //extrapPos=grid_map::Position(x,y);

  Eigen::Vector2d ncP=next-currentPos;
  //Angle between robot and subgoal
  double subgoalAngle=normAngle(atan2(ncP.y(),ncP.x()));

  bool success;
  grid_map::GridMap  sub=map.getSubmap(extrapPos, grid_map::Length(LENGTH,LENGTH), success);
  
  if(!success)
    ROS_ERROR("Unable to get Submap");
  
  sub["utility"].setConstant(-DBL_MAX);
  sub["motion"].setConstant(-DBL_MAX);
  sub["env"].setConstant(-DBL_MAX);
  sub["pair"].setConstant(-DBL_MAX);

  Eigen::Vector2d direction(cos(yaw),sin(yaw));
  int counter=0;
  
  grid_map::Position currentPartner=grid_map::Position(partnerPose.getOrigin().getX(),partnerPose.getOrigin().getY());
 
  std::vector<Eigen::Vector2d> objs;
  if(sub.isInside(currentPartner))
    objs.push_back(currentPartner);
  //Preprocess traversability layer
  if(sub["traversability"].any()){
    grid_map::Matrix& data = sub["traversability"];
    for (grid_map::GridMapIterator iterator(sub);!iterator.isPastEnd(); ++iterator) {
      //const grid_map::Index index(*iterator);
      if(sub.at("traversability",*iterator)){
	grid_map::Position pos;
  	sub.getPosition(*iterator,pos);
	objs.push_back(pos);
      }
    }
    if(!objs.size())
      ROS_ERROR("objects shouldn't be empty");
  }

  //Calculate utilities at each grid cell
  //for (grid_map::Index it: findTraver(sub,extrapPos)) {

  for(grid_map::GridMapIterator itr(sub); !itr.isPastEnd(); ++itr){
    grid_map::Index it=*itr;
    grid_map::Position pos;
    sub.getPosition(it,pos);
    Eigen::Vector2d pcP=pos-currentPos;
    
    if(direction.dot(pcP)<0.1){
      continue;
    }
    double angle=normAngle(atan2(pcP.y(),pcP.x()));
    double vel=pcP.norm()/delta_time;

    //Motion Factors
    double motion=util_motion(pos,yaw,dv,delta_time,angle,vel);
    sub.at("motion",it)=motion;

    //Environment Factors
    double env=util_env(pos,subgoalAngle,angle,objs);
    sub.at("env",it)=env;


    double utility=motion+env;

    sub.at("utility",it)=utility;
  }
  return sub;

}


grid_map::GridMap Utility::simplePartnered(double dv, double dw, tf::Pose currentPose, grid_map::GridMap map, grid_map::Position next, double pdv, double pdw, tf::Pose partnerPose)
{
  double max = -DBL_MAX;;
  tf::Vector3 center = currentPose.getOrigin();
  double yaw = tf::getYaw(currentPose.getRotation());  
  double theta = normAngle((dw * delta_time)+yaw);

  double x,y;
  //x = dv * cos(theta) * delta_time + center.getX();
  //y = dv * sin(theta) * delta_time + center.getY();

  grid_map::Position currentPos=grid_map::Position(center.getX(),center.getY());
  grid_map::Position extrapPos=currentPos;
  //extrapPos=grid_map::Position(x,y);

  Eigen::Vector2d ncP=next-currentPos;
  //Angle between robot and subgoal
  double subgoalAngle=normAngle(atan2(ncP.y(),ncP.x()));

  bool success;
  grid_map::GridMap sub=map.getSubmap(extrapPos, grid_map::Length(LENGTH,LENGTH), success);
  if(!success)
    ROS_ERROR("Unable to get Submap");
  
  sub["utility"].setConstant(0.0);
  sub["motion"].setConstant(0.0);
  sub["env"].setConstant(0.0);
  sub["pair"].setConstant(0.0);

  Eigen::Vector2d direction(cos(yaw),sin(yaw));
  int counter=0;
  

  //Predicted partner position in the next timestep
  double pYaw=tf::getYaw(partnerPose.getRotation());
  double ptheta = normAngle((pdw * delta_time)+pYaw);
  double pX=pdv*cos(pYaw) *delta_time+partnerPose.getOrigin().getX();
  double pY=pdv*sin(pYaw) *delta_time+partnerPose.getOrigin().getY();
  grid_map::Position partnerPos=grid_map::Position(pX,pY);
  grid_map::Position currentPartner=grid_map::Position(partnerPose.getOrigin().getX(),partnerPose.getOrigin().getY());

  std::vector<Eigen::Vector2d> objs;
  // if(sub.isInside(currentPartner))
  //   objs.push_back(currentPartner);
  // //Preprocess traversability layer
  if(sub["traversability"].any()){
    grid_map::Matrix& data = sub["traversability"];
    for (grid_map::GridMapIterator iterator(sub);!iterator.isPastEnd(); ++iterator) {
      //const grid_map::Index index(*iterator);
      if(sub.at("traversability",*iterator)!=0){
  	grid_map::Position pos;
  	sub.getPosition(*iterator,pos);
  	objs.push_back(pos);
      }
    }
    if(!objs.size())
      ROS_ERROR("objects shouldn't be empty");
  }



  //Calculate utilities at each grid cell
  double resolution=sub.getResolution();
  for (grid_map::GridMapIterator iterator(sub);!iterator.isPastEnd(); ++iterator) {
    
    grid_map::Position pos;
    sub.getPosition(*iterator,pos);
    Eigen::Vector2d pcP=pos-currentPos;
    if(direction.dot(pcP)<0.1&&pcP.norm()>2*resolution){
      sub.at("utility",*iterator)=-DBL_MAX;
      sub.at("motion",*iterator)=-DBL_MAX;
      sub.at("env",*iterator)=-DBL_MAX;
      sub.at("pair",*iterator)=-DBL_MAX;
      continue;
    }
    
    double angle=normAngle(atan2(pcP.y(),pcP.x()));
    double vel=pcP.norm()/delta_time;

    //Motion Factors
    double motion=util_motion(pos,yaw,dv,delta_time,angle,vel);
    sub.at("motion",*iterator)=motion;

    //Environment Factors
    double env=util_env(pos,subgoalAngle,angle,objs);
    sub.at("env",*iterator)=env;

    double partner=util_relative(pos, currentPos, vel, currentPartner, partnerPos, ptheta,pdv);
    sub.at("pair",*iterator)=partner;

    // double utility=motion+env+partner;
    double utility=motion+env+partner;
    sub.at("utility",*iterator)=utility;
  }
  return sub;

}



grid_map::GridMap Utility::doubleUtil(grid_map::GridMap self, grid_map::GridMap partner,tf::Pose currentPose,tf::Pose partnerPose){
  grid_map::Position currentPos=grid_map::Position(currentPose.getOrigin().getX(),currentPose.getOrigin().getY());
  grid_map::Position partnerPos=grid_map::Position(partnerPose.getOrigin().getX(),partnerPose.getOrigin().getY());
  //iterate through each cell of both gridmaps
  for(grid_map::GridMapIterator itr1(self); !itr1.isPastEnd(); ++itr1){
    if(self.at("utility",*itr1)==-DBL_MAX){
      self.at("pair",*itr1)=-DBL_MAX;
      continue;
    }
    //Prelimary calculations of velocity, displacement and angle to travel to that cell (for relative utility)
    grid_map::Position selfPos;
    self.getPosition(*itr1,selfPos);
    Eigen::Vector2d disp=selfPos-currentPos;
    
    double vel=disp.norm()/delta_time;
    double maxUtil=-DBL_MAX;
    double maxRel;
    double angle=normAngle(atan2(disp.y(),disp.x()));
    //Get self utility for the cell
    double selfUtil=self.at("utility",*itr1);
    //For each of self's potential 1-step destinations, iterate through each of the partner's potential 1-step destinations
    for(grid_map::GridMapIterator itr2(partner); !itr2.isPastEnd(); ++itr2){
 
      if(partner.at("utility",*itr2)==-DBL_MAX){
	continue;
      }
      grid_map::Position pPos;
      partner.getPosition(*itr2,pPos);
      //calculate utility for relative displacement 
      Eigen::Vector2d relativeDisp=selfPos-pPos;
      double dist=relativeDisp.norm();
      double distUtil=relativeDist.util(relativeDisp.norm());

      Eigen::Vector2d pDisp=(pPos-partnerPos);

      //calculate utility for relative velocity
      double pVel=pDisp.norm()/delta_time;
      double velUtil=relativeVel.util(pVel-vel);
      

      //Calculate utiltiy for relative angle
      double anglePartner=atan2(pDisp.y(),pDisp.x());
      double angleBtwn=atan2(relativeDisp.y(),relativeDisp.x());//Angle of the vector between predicted positions
      double relAngle=normAngle(anglePartner-angleBtwn);
      double angUtil=relativeAngle.util(relAngle);


      //calculate self's utility for the pair of cells
      double relUtil=angUtil+velUtil+distUtil;
      double totUtil=relUtil+partner.at("utility",*itr2);
 
      if(totUtil>maxUtil){
      	maxUtil=totUtil;
	maxRel=relUtil;
      }
    }
    self.at("pair",*itr1)=maxRel;
    
    self.at("utility",*itr1)+=maxUtil;
  }
  return self;
}

*/

//For use with Frene
std::vector< std::vector< std::vector<double> > > Utility::trajectoryUtil(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn, std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut, Eigen::Vector2d next_subgoal, double yaw, std::vector<Eigen::Vector2d> objs,double& max, double& min, std::string ns ){
  trajectoryArrayOut.clear();
  double maxUtil=-DBL_MAX;
  double minUtil=DBL_MAX;
  std::vector< std::vector< double > > opt;

  double max_ang, max_diff ; 
  double angleToNext ;
  double x, y, theta;
  //std::cerr << "  subgoal: " << next_subgoal.x() << " " << next_subgoal(1) ;
  
  #pragma omp parallel 
  {
    for( std::vector<std::vector< std::vector< double > > >::iterator itr =trajectoryArrayIn.begin(); itr != trajectoryArrayIn.end(); itr++ ){  

      #pragma omp single nowait
      {
	std::vector<double> prev[2];
	prev[0]=itr->back();
	prev[1]=(*itr)[itr->size()/2];
	double utility=0;
	std::vector< std::vector<double> > tmp;
	
	for(auto back: prev){
	  x=back[0];
	  y=back[1];
	  theta=  normAngle(  back[3]  ) ;
	  double dtTheta=back[4];
	  double vx=back[5];      
	  double ax=back[6];      
	  double kx=back[7]; 
	  

	  //Motion Utilities
	  utility+=velocity.util(vx);
	  utility+=angularVel.util(dtTheta);
	  utility+=accel.util(ax);
	
	  //Environmental Utilities
	  Eigen::Vector2d position(x,y);
	  
	  //HERE fixed Angle by Luis
	  Eigen::Vector2d toNext=next_subgoal-position;
	  angleToNext = normAngle(atan2(toNext.y(),toNext.x()));
	  utility+=subgoalAngle.util(angleToNext-theta);

	  //if( strcmp ( ns.c_str(), "v1" ) == 0 )
	  //std::cerr << "\nUtil current: " << x << " " << y << " " << theta*180/M_PI << " subgoal: " << next_subgoal.x() << " " << next_subgoal(1) << " "  << angleToNext*180/M_PI << " " <<  (angleToNext-theta) * 180/M_PI << " ";
	  
	  if( !objs.empty()){
	    double obstDist=DBL_MAX;
	    for(std::vector<Eigen::Vector2d>::iterator obj=objs.begin(); obj!=objs.end(); obj++)
	      if((position-*obj).norm()<obstDist)
		obstDist=(position-*obj).norm();
	    utility+=obstacleDist.util(obstDist);
	  }
	} // end for back
	
	utility/=2;
	//for( std::vector< std::vector< double > >::iterator itr2=itr->begin(); itr2 != itr->end(); itr2++ ){
	for(int i=0; i<itr->size(); i++){
	  std::vector<double> point((*itr)[i]);
	  point.push_back(utility);
	  tmp.push_back(point);
	}

#pragma omp critical
	{
	  trajectoryArrayOut.push_back(tmp);
	  if(utility>maxUtil){
	    opt=(*itr);
	    maxUtil=utility;
	    max_ang =   angleToNext*180/M_PI;
	    max_diff = (angleToNext-theta) * 180/M_PI ;
	  }
	  if(utility<minUtil)
	    minUtil=utility;
	  } //end omp critical
	
	  } //end single no wait
	  }  //end omp parallel 
    }

    //if( strcmp ( ns.c_str(), "v1" ) == 0 )
    //std::cerr << "  Util: " << x << " " << y << " " << theta*180/M_PI << " subgoal: " << next_subgoal.x() << " " << next_subgoal(1) << " "  << max_ang << " " <<  max_diff;

  max=maxUtil;
  min=minUtil;
  std::vector< std::vector< std::vector<double> > > path{opt};
  return path;
}





std::vector< std::vector< std::vector<double> > > Utility::trajectoryStop(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn,
									  std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut, 
									  Eigen::Vector2d pos, double yaw
									  )
{
  trajectoryArrayOut.clear();
  double maxUtil=-DBL_MAX;

  std::vector< std::vector< double > > opt;
  #pragma omp parallel
  {
    for( std::vector< std::vector< std::vector< double > > >::iterator itr=trajectoryArrayIn.begin(); itr != trajectoryArrayIn.end(); itr++ ){        
      #pragma omp single nowait
      {
      std::vector< std::vector<double> > tmp(*itr);
      std::vector<double> back=tmp.back();
      double x=(back)[0];
      double y=(back)[1];
      double theta=(back)[3];
      double dtTheta=(back)[4];
      double vx=(back)[5];     
      double utility=back[21];
      Eigen::Vector2d position(x,y);
      Eigen::Vector2d dist=(position-pos);
      utility+=2*relativeDist.util(dist.norm());
      utility+=2*relativeVel.util(vx);
      double angleBtwn=atan2(dist.y(),dist.x());
      double angleBtwn2=atan2(-dist.y(),-dist.x());
      utility+=relativeAngle.util(abs(normAngle(theta-angleBtwn2)));
      utility+=relativeAngle.util(abs(normAngle(yaw-angleBtwn)));
      #pragma omp critical
	{
	  trajectoryArrayOut.push_back(tmp);
	  if(utility>maxUtil){
	    opt=(*itr);
	    maxUtil=utility;
	  }
	}
      }
    }
  }
  std::vector< std::vector< std::vector<double> > > path{opt};
  return path;
}


std::vector< std::vector< std::vector<double> > > Utility::combinedTrajectory(std::vector< std::vector< std::vector<double> > >& trajectoryArraySelf, 
									      std::vector< std::vector< std::vector<double> > >& trajectoryArrayPartner, 
									      std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut, 
									      double& max, double& min)
{
  trajectoryArrayOut.clear();
  double maxUtil=-DBL_MAX;
  double minUtil=DBL_MAX;
  double rel_dist, rel_angle, rel_vel;
  double angleBtwn, angleBtwn2;
  double thetaP;
  double dtThetaP;
  double vP;
  Eigen::Vector2d dist;
  double max_util_rel_dist; 

  
  std::vector< std::vector< double > > opt;
  //#pragma omp parallel
  //{
    for( std::vector< std::vector< std::vector< double > > >::iterator itr=trajectoryArraySelf.begin(); itr != trajectoryArraySelf.end(); itr++ ){        
      //#pragma omp single nowait
      //{
      std::vector< std::vector<double> > tmp(*itr);
      double utility=-DBL_MAX;
      std::vector<double> back=tmp.back();
      double x=(back)[0];
      double y=(back)[1];
      double theta=(back)[3];
      double dtTheta=(back)[4];
      double vx=(back)[5];      
      for( std::vector< std::vector< std::vector< double > > >::iterator itr2=trajectoryArrayPartner.begin(); itr2 != trajectoryArrayPartner.end(); itr2++ ){        
	std::vector<double> backP=(*itr2).back();
	double xP=(backP)[0];
	double yP=(backP)[1];
	thetaP=(backP)[3];
	double dtThetaP=(backP)[4];
	vP=(backP)[5];      
	dist << x-xP, y-yP ; 
	double trajUtil=0;

	rel_dist = Dist_Between_Points( x, y,  xP, yP ) ;

	// Rel dist
	//std::cerr << "\n dist: " << x << " " << y << " P: " << xP << " " << yP << "  dist: " << dist[0]  << " " << dist.y() << " " << rel_dist  << " " << dist.norm();
	
	//std::cerr << "\n dist: " << dist.norm() << " ang: " << rel_angle * 180 / M_PI << " rel_vel: " << rel_vel ; 
	//relative distance
	//trajUtil+=2*relativeDist.util(dist.norm());
	trajUtil+=2*relativeDist.util(dist.norm());
	//std::cerr << "\n dist: " << dist.norm(); 
	//relative angle
	
	angleBtwn=atan2(dist.y(),dist.x());
	angleBtwn2=atan2(-dist.y(),-dist.x());
	trajUtil+=relativeAngle.util(abs(normAngle(theta-angleBtwn2)));
	trajUtil+=relativeAngle.util(abs(normAngle(thetaP-angleBtwn)));
	
	if(dist.norm()<2&&abs(normAngle(theta-angleBtwn2))>=M_PI/4&&abs(normAngle(theta-angleBtwn2))<=3*M_PI/4){
	  //relative velocity
	  trajUtil+=2*relativeVel.util(vx-vP);
	  //relativeAngularUtil
	  //trajUtil+=2*relativeVel.util(dtTheta-dtThetaP);
	}
	//non-relative utilities
	trajUtil+=back.at(21)+backP.at(21);
	//#pragma omp critical
	//{
	  if(trajUtil>utility)
	    utility=trajUtil;
	  //}
	  
      } //for itr2
      
      //#pragma omp critical
      //{
	back[21]=utility;
	trajectoryArrayOut.push_back(tmp);
	
	if(utility>maxUtil){
	  opt=tmp;
	  maxUtil=utility;

	  max_util_rel_dist = dist.norm();

	    //rel_dist  = dist.norm() ;
	  rel_angle = abs(normAngle(thetaP-angleBtwn)) ;
	  rel_vel   = vx-vP ;
			      
	}  // if utility
	if(utility<minUtil)
	  minUtil=utility;

	//}  //omp critical
	//} //omp single nowait
    } // for itr1
    //}  //omp parallel

  std::cerr << "\n dist: " << max_util_rel_dist << " ang: " << rel_angle * 180 / M_PI << " rel_vel: " << rel_vel ; 
  max=maxUtil;
  min=minUtil;
  std::vector< std::vector< std::vector<double> > > path{opt};
  return path;
}


















std::vector< std::vector< std::vector<double> > > Utility::LuiscombinedTrajectory(std::vector< std::vector< std::vector<double> > >& trajectoryArraySelf, 
									      std::vector< std::vector< std::vector<double> > >& trajectoryArrayPartner, 
									      std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut, 
										  double& max, double& min,
										  Eigen::Vector2d selfPos, double selfYaw, Eigen::Vector2d partnerPos, double partnerYaw)
{
  trajectoryArrayOut.clear();
  double maxUtil=-DBL_MAX;
  double minUtil=DBL_MAX;
  double rel_dist, rel_angle, rel_vel;
  double angleBtwn, angleBtwn2;
  double thetaP;
  double dtThetaP;
  double vP;
  Eigen::Vector2d dist;
  double max_util_rel_dist;
  double max_util_rel_angle; 

  
  std::vector< std::vector< double > > opt;
  //#pragma omp parallel
  //{
    for( std::vector< std::vector< std::vector< double > > >::iterator itr=trajectoryArraySelf.begin(); itr != trajectoryArraySelf.end(); itr++ ){        
      //#pragma omp single nowait
      //{
      std::vector< std::vector<double> > tmp(*itr);
      double utility=-DBL_MAX;
      std::vector<double> back=tmp.back();
      double x=(back)[0];
      double y=(back)[1];
      double theta=(back)[3];
      double dtTheta=(back)[4];
      double vx=(back)[5];      
      for( std::vector< std::vector< std::vector< double > > >::iterator itr2=trajectoryArrayPartner.begin(); itr2 != trajectoryArrayPartner.end(); itr2++ ){        
	std::vector<double> backP=(*itr2).back();
	double xP=(backP)[0];
	double yP=(backP)[1];
	thetaP=(backP)[3];
	double dtThetaP=(backP)[4];
	vP=(backP)[5];      
	dist << x-xP, y-yP ; 
	double trajUtil=0;

	rel_dist = Dist_Between_Points( x, y,  xP, yP ) ;

	// Rel dist
	//std::cerr << "\n dist: " << x << " " << y << " P: " << xP << " " << yP << "  dist: " << dist[0]  << " " << dist.y() << " " << rel_dist  << " " << dist.norm();
	
	//std::cerr << "\n dist: " << dist.norm() << " ang: " << rel_angle * 180 / M_PI << " rel_vel: " << rel_vel ;


	//relative distance
	//trajUtil+=2*relativeDist.util(dist.norm());
	trajUtil+=2*relativeDist.util(dist.norm());
	//std::cerr << "\n dist: " << dist.norm(); 
	//relative angle
	
	angleBtwn=atan2(dist.y(),dist.x());
	angleBtwn2=atan2(-dist.y(),-dist.x());

	//row.t2 - math.atan2(row.y1 - row.y2,  row.x1 - row.x2
	rel_angle = normAngle ( normAngle( partnerYaw ) - normAngle ( atan2( dist.y(), dist.x() ) )) ;

       
	std::cerr << "\n angle: " << normAngle(partnerYaw) * 180/M_PI << " " <<    normAngle(atan2( dist.y(), dist.x()) ) * 180/M_PI << " " << rel_angle * 180 / M_PI ;
	
	trajUtil+=relativeAngle.util(abs(normAngle(theta-angleBtwn2)));
	trajUtil+=relativeAngle.util(abs(normAngle(thetaP-angleBtwn)));
	
	if(dist.norm()<2&&abs(normAngle(theta-angleBtwn2))>=M_PI/4&&abs(normAngle(theta-angleBtwn2))<=3*M_PI/4){
	  //relative velocity
	  trajUtil+=2*relativeVel.util(vx-vP);
	  //relativeAngularUtil
	  //trajUtil+=2*relativeVel.util(dtTheta-dtThetaP);
	}
	//non-relative utilities
	trajUtil+=back.at(21)+backP.at(21);
	//#pragma omp critical
	//{
	  if(trajUtil>utility)
	    utility=trajUtil;
	  //}
	  
      } //for itr2
      
      //#pragma omp critical
      //{
	back[21]=utility;
	trajectoryArrayOut.push_back(tmp);
	
	if(utility>maxUtil){
	  opt=tmp;
	  maxUtil=utility;

	  max_util_rel_dist = dist.norm();
	  max_util_rel_angle = rel_angle;

	    //rel_dist  = dist.norm() ;
	  rel_angle = abs(normAngle(thetaP-angleBtwn)) ;
	  rel_vel   = vx-vP ;
			      
	}  // if utility
	if(utility<minUtil)
	  minUtil=utility;

	//}  //omp critical
	//} //omp single nowait
    } // for itr1
    //}  //omp parallel

  std::cerr << "\n dist: " << max_util_rel_dist << " ang: " << max_util_rel_angle * 180 / M_PI << " rel_vel: " << rel_vel ; 
  max=maxUtil;
  min=minUtil;
  std::vector< std::vector< std::vector<double> > > path{opt};
  return path;
}






