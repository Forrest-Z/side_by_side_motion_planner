#ifndef FRENE_H
#define FRENE_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include "spline.h"
//classes and structures
#define FRENESTATESIZE 12
class FreneState
{
 private:
 public:
  double s;
  double d;
  double ts;
  double td;
  double dts;
  double ddts;
  double dddts;
  double dtd;
  double ddtd;
  double dddtd;
  double sumdddts;
  double sumdddtd;
  
  FreneState(void){
    s = 0;
    d = 0;
    ts = 0;
    td = 0;
    dts = 0;
    ddts = 0;
    dddts = 0;
    dtd = 0;
    ddtd = 0;
    dddtd = 0;
    sumdddts = 0;
    sumdddtd = 0;
  };
  
  FreneState& operator = ( const FreneState& rhs )
  {
    s  = rhs.s;
    d  = rhs.d;
    ts  = rhs.ts; 
    td  = rhs.td; 
    dtd = rhs.dtd;
    ddtd = rhs.ddtd;
    dddtd = rhs.dddtd;
    dts = rhs.dts;
    ddts = rhs.ddts;
    dddts = rhs.dddts;
    sumdddtd = rhs.sumdddtd;    
    sumdddts = rhs.sumdddts;    

    return *this;
  }
  
  static int size(void){
    return FRENESTATESIZE;
  }
};


class LineState
{
 private:
 public:
  Eigen::VectorXd rr;
  Eigen::VectorXd nr;
  Eigen::VectorXd tr;
  Eigen::VectorXd dstr;
  double dskr;
  double kr;//curvature
  double thetar;
  
  LineState(void){
    rr.resize(2);
    nr.resize(2);
    tr.resize(2);
    dstr.resize(2);
    dskr = 0;
    kr = 0;
    thetar = 0;
  };
  
  LineState& operator = ( const LineState& rhs )
  {
    rr = rhs.rr;
    nr = rhs.nr;
    tr = rhs.tr;
    dstr = rhs.dstr;
    dskr = rhs.dskr;
    kr = rhs.kr;
    thetar = rhs.thetar;
    
    return *this;
  }  
  


};



class Frene
{
 private:  
  void computeLineStateAt( double s, LineState& state, spline_struct& splineX, spline_struct& splineY );
  void computeLineStateAtInterp( double s, LineState& state, spline_struct& splineX, spline_struct& splineY );
  void interpXYListFromSpline( //tk::spline& splineX, tk::spline& splineY, 
			       std::vector< std::vector<double> >& xyInterpList, double interpTick );
  void interpXYListFromSpline( //tk::spline& splineX, tk::spline& splineY,
			      std::vector< std::vector<double> >& xyInterpList, double interpTick, double totalArcLength, spline_struct& splineX, spline_struct& splineY );
  double computeSplineFromXYList( std::vector< std::vector<double> >& xyList, spline_struct& splineX, spline_struct& splineY   ); 
 
  void computeSplineFromXYListWithResamplingDenseData( std::vector< std::vector<double> >& xyList, double resamplingThreshold, 
						       std::vector< std::vector<double> >& xyResamplingListReturn,
						       spline_struct& splineX, spline_struct& splineY );
						       

  bool isSmallerThanOrApproxEqual( double val1, double val2, double eps );  
  double _totalArcLength;
  
 public:
  Frene(std::vector< std::vector<double> >& laneListIn, double resamplingThreshold, /*std::vector< std::vector<double> >& xyResamplingList,*/ spline_struct& splineX, spline_struct& splineY, std::string ns,  bool self );
  
  void convertFromCartesianState(double t, double x, double y, double theta, double vx, double ax, double kx, double dtTheta, FreneState& freneState, double interpTick, spline_struct& splineX, spline_struct& splineY, Eigen::Vector2d &next_subgoal,  double distance_to_subgoal_ahead, std::string ns, bool self);
  void convertFromFreneState(FreneState& freneState, double& x, double& y, double& theta, double& vx, double& ax, double& kx, double& dtTheta, spline_struct& splineX, spline_struct& splineY);

  void filterByFindMostConservativeOne( std::vector< std::vector< std::vector<double> > >& trajectoryArray, 				  
					std::vector< std::vector< std::vector<double> > >& optimalTrajectoryArray
					);			      

  void computeOptimalTrajectory( std::vector< std::vector< std::vector<double> > >& trajectoryArray, 
				 std::vector< std::vector< std::vector<double> > >& paramArray, 
				 std::vector< std::vector< std::vector<double> > >& optimalTrajectoryArray, 
				 std::vector< std::vector< std::vector<double> > >& optimalParamArray, 
				 double kjlong, double ktlong, double kplong, 
				 double kjlat, double ktlat, double kplat, 				     
				 double klong, double klat,
				 double desiredSpeed, double desiredLatPosition
				 );
  void convertMergedTrajectoryArrayToCartesian( std::vector< std::vector< std::vector<double> > >& trajectoryArrayList, 
						std::vector< std::vector< std::vector<double> > >& trajectoryArrayCartesianList, spline_struct& splineX, spline_struct& splineY );
  void generateLateralAndLongitudinalTrajectoryVelocityKeeping( FreneState& initialState, 
								std::vector< std::vector< std::vector<double> > >& trajectoryArrayList,
								std::vector< std::vector< std::vector<double> > >& trajectoryParamList,
								double timeMinSec, double timeMaxSec, double timeDeltaSec, 
								double desiredSpeed, double desiredSpeedMinus, double desiredSpeedPlus, double desiredSpeedDelta,
								double desiredLatPosition, double desiredLatPositionMinus, double desiredLatPositionPlus, double desiredLatPositionDelta, double tickTime);
  void generateLateralAndLongitudinalTrajectoryFollowing( FreneState& initialState,
							  std::vector< std::vector< std::vector<double> > >& trajectoryArrayList,
							  std::vector< std::vector< std::vector<double> > >& paramArrayList,
							  double desiredLongSpeed, double desiredLongAcceleration, 
							  double timeMinSec, double timeMaxSec, double timeDeltaSec, 
							  double desiredLongPosition, double desiredLongPositionMinus, double desiredLongPositionPlus, double desiredLongPositionDelta,
							  double desiredLatPosition, double desiredLatPositionMinus, double desiredLatPositionPlus, double desiredLatPositionDelta,
							  double tickTime
							  );
  void generateLateralAndLongitudinalTrajectoryStopping( FreneState& initialState,
							 std::vector< std::vector< std::vector<double> > >& trajectoryArrayList,
							 std::vector< std::vector< std::vector<double> > >& paramArrayList,
							 double desiredLongSpeed, double desiredLongAcceleration, 
							 double timeMinSec, double timeMaxSec, double timeDeltaSec, 
							 double desiredLongPosition, double desiredLongPositionMinus, double desiredLongPositionPlus, double desiredLongPositionDelta,
							 double desiredLatPosition, double desiredLatPositionMinus, double desiredLatPositionPlus, double desiredLatPositionDelta,
							 double tickTime
							 );
  
  void computeInitialStateFromParam(double t, std::vector< std::vector< std::vector<double> > >& paramArray, FreneState& state );
  
  void computeInitialStateFromPreviousTrajectory( double xCurrent, double yCurrent, double vxCurrent, std::vector< std::vector< std::vector< double > > >& previousTrajectoryArray, 
						  double& x, double& y, double& t, double& theta, double& dtTheta, double& vx, double& ax, double& kx );    

  static void multiplyAmplitude(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn, 
				std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut,
				double amplitude);

  void filterBySafety(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn, 
		      std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut,
		      double roadMarginRight,  
		      double roadMarginLeft,     
		      double maxCentrifugalForce,
		      double maxAcceleration,
		      double maxCurvature		   
		      );
  void filterByCollision(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn, 
			 std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut,
			 grid_map::GridMap map,
			 double Length,
			 double Width,
			 Eigen::Vector2d currentPos,
			 double yaw
			 );

  void filterByDesiredLatPosition(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn, 
				  std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut,
				  double desiredLatPosition
				  ); 
 void filterByDesiredLongPosition(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn, 
				  std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut,
				  double desiredLongPosition
				  );
  void filterByDesiredSpeed(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn, 
			    std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut,
			    double desiredSpeed
			    );
  void filterByComfort(std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn, 
		       std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut			    
		       );
  void computeInitialStateFromPreviousTrajectoryWithSplineResampling(double xCurrent, double yCurrent, 
								      std::vector< std::vector< std::vector< double > > >& previousTrajectoryArray, 
								      double& x, double& y, double& t, double& theta, double& dtTheta, double& vx, double& ax, double& kx,
								     double resamplingTickTime,
								     double timeFromPreviousCompute,
								     bool useTimeMode

								      );

  void computeInitialStateDirectFromPreviousTrajectoryWithSplineResampling(double xCurrent, double yCurrent, 
									   std::vector< std::vector< std::vector< double > > >& previousTrajectoryArray, 
									   FreneState& freneState,
									   double resamplingTickTime
									   );    
  
  void filterByOptimalityCostFromCartesianTrajectoryArray( std::vector< std::vector< std::vector<double> > >& trajectoryArray, 				  
							   std::vector< std::vector< std::vector<double> > >& optimalTrajectoryArray, 
							   double kjlong, double ktlong, double kplong, 
							   double kjlat, double ktlat, double kplat, 				     
							   double klong, double klat,
				 			   int freneMode,
							   double desiredSpeed, double desiredLatPosition,
							   double desiredLongPosition
							   );

  static void setAsInvalidTrajectory( std::vector< std::vector< std::vector< double > > >& path );
  static bool isInvalidTrajectory( std::vector< std::vector< std::vector< double > > >& path );
  static void computeTrajectoryZero( std::vector< std::vector< std::vector<double> > >& previousOptimalTrajectory, 
				     std::vector< std::vector< std::vector<double> > >& trajectoryZero,
				     double trajectoryLength,
				     int numOfPoints
				     );
  static void computeTrajectorySmoothStopping( std::vector< std::vector< std::vector<double> > >& previousOptimalTrajectory, 
					   std::vector< std::vector< std::vector<double> > >& trajectoryZero,
					   double trajectoryLength,
					   double smoothnessFactor,
					   int numOfPoints );


  static void addDeadZone( std::vector< std::vector< std::vector<double> > >& trajectoryArrayIn, 				  
			  std::vector< std::vector< std::vector<double> > >& trajectoryArrayOut,
			  double sDeadZoneLength    );
  
};

#endif
