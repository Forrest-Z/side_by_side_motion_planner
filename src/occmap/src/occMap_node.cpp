#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <velodyne_pointcloud/point_types.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#define DIST 30
ros::Publisher publisher;
double _leafSize, _width, _dist;
std::vector<grid_map::Position> points;
Eigen::Vector2d _currentPose;
grid_map::GridMap occMap({"traversability"});
ros::Publisher mapPublisher, vizPub;


void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr& input){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_y (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr flat (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input,*cloud);

  //std::cout << "inside callback: " << input->height << " " << input->width << std::endl;

  //apply a box filter
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(-_dist, -_width/2, -10, 1.0));
  boxFilter.setMax(Eigen::Vector4f( _dist, _width/2, 10.5, 1.0));
  boxFilter.setInputCloud(cloud);
  boxFilter.filter(*filter_y);


  //voxelize to downsample
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud(filter_y);
  vox.setLeafSize(_leafSize,_leafSize,20);
  vox.filter(*cloud);
  
  
  //orthographic projection to X-Y plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*flat);


  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*flat, output);
  publisher.publish(output);
  points.clear();
  occMap["traversability"].setConstant(0);
  for (int i = 0; i < flat->points.size(); i++)
  {
      double x=flat->points[i].x;
      double y=flat->points[i].y;

      grid_map::Position point(x,y);
      if(occMap.isInside(point))
	occMap.atPosition("traversability", point) = 1.0;
  }
  grid_map_msgs::GridMap msg;
  nav_msgs::OccupancyGrid occ;
  grid_map::GridMapRosConverter::toMessage(occMap, msg);
  grid_map::GridMapRosConverter::toOccupancyGrid(occMap, "traversability",0,1,occ);
  mapPublisher.publish(msg);
  vizPub.publish(occ);
}

void pointcloud1_callback(const sensor_msgs::PointCloudConstPtr& input){
  sensor_msgs::PointCloud2 input2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_y (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::convertPointCloudToPointCloud2(*input,input2);

  //std::cout << "inside callback1: " << input->points.size() << "  p2: " << input2.width << std::endl;

  pcl::fromROSMsg(input2,*cloud);

  //std::cout << "inside callback2: " << cloud->points.size() << std::endl;
  //for (size_t i = 0; i < cloud->points.size (); ++i)
  //std::cout << cloud->points[i].x <<  " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
  //std::cout << cloud->points[i].x <<  " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;  

  //apply a box filter
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(-_dist, -_width/2, -10, 1.0));
  boxFilter.setMax(Eigen::Vector4f( _dist,  _width/2,  10, 1.0));
  
  //boxFilter.setMin(Eigen::Vector4f(-20, -20, -10, 1.0));
  //boxFilter.setMax(Eigen::Vector4f( 20,  20,  10, 1.0));
  boxFilter.setInputCloud(cloud);
  boxFilter.filter(*filter_y);

  //std::cout << "inside callback3: " << filter_y->points.size() << "  width: " << _width << "  dist: " <<_dist <<  std::endl;

  //voxelize to downsample
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud(filter_y);
  vox.setLeafSize(_leafSize,_leafSize,20);
  vox.filter(*cloud);

  //std::cout << "inside callback4: " << cloud->points.size() << std::endl;

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  publisher.publish(output);
  points.clear();
  occMap["traversability"].setConstant(0);

  
  for (int i = 0; i < cloud->points.size(); i++)
  {
    //double x=cloud->points[i].x;
    //double y=cloud->points[i].y;
    //grid_map::Position point(x,y);
    grid_map::Position point( cloud->points[i].x, cloud->points[i].y );
      if(occMap.isInside(point))
	occMap.atPosition("traversability", point) = 1.0;
  }
  grid_map_msgs::GridMap msg;
  nav_msgs::OccupancyGrid occ;
  grid_map::GridMapRosConverter::toMessage(occMap, msg);
  grid_map::GridMapRosConverter::toOccupancyGrid(occMap, "traversability",0,1,occ);
  mapPublisher.publish(msg);
  vizPub.publish(occ);
}

static void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  _currentPose[0]=msg->pose.position.x;
  _currentPose[1]=msg->pose.position.y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occmap_node");


  ros::NodeHandle n("~") ;
  ros::NodeHandle nh ;
  std::string occMapName,publish,subscribe, occMapFrame;
  double leafSize, dist, width;
  bool scanFromPointcloud1;
  n.param<std::string>("occMapTopicName",occMapName, "occmap");
  n.param<std::string>("occMapFrame",occMapFrame, "base_link");
  n.getParam("viz",publish);
  n.getParam("cloud",subscribe);
  n.param<double>("leafSize",leafSize,0.2);
  n.param<double>("dist",dist,1.4);
  n.param<double>("width",width,1.4);
  n.param<bool>("pointcloud1_msg",scanFromPointcloud1,false);
  _leafSize=leafSize;
  _dist=dist;
  _width=width;
  ros::Subscriber pointcloudSubscriber,pointcloud2Subscriber;
  if(scanFromPointcloud1){
    pointcloudSubscriber = nh.subscribe(subscribe, 1, pointcloud1_callback);
    ROS_INFO("Using pointcloud1");
  }
  else{
    pointcloud2Subscriber=nh.subscribe(subscribe,1,pointcloud2_callback);
    ROS_INFO("Using scan from pointcloud");
  }
  publisher = nh.advertise<sensor_msgs::PointCloud2>(publish, 1, true);
  mapPublisher=nh.advertise<grid_map_msgs::GridMap>(occMapName, 1, false);
  ros::Subscriber currentPoseSubscriber = nh.subscribe("filtered_ndt_current_pose", 1, currentPoseCallback); 
  vizPub=nh.advertise<nav_msgs::OccupancyGrid>("occMapViz",1,false);


  occMap.setFrameId(occMapFrame);
  grid_map::Position mapPos(0,0);
  occMap.setGeometry(grid_map::Length(dist,width/2),leafSize, mapPos);
  occMap.add("traversability",0.0);
  ros::spin();  
  return 0;
}
