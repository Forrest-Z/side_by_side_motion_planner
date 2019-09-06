#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <safety_limiter/Status.h>
#include <dynamic_object_msgs/DynamicObjectArray.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <eigen3/Eigen/Core>
#include <pcl/common/transforms.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Core>

//tf_conversions/tf_eigen.h
   
class limiter
{
public:
	limiter();
	~limiter();
	void spin();
private:
  tf::TransformListener tf_listener;
  
  std::string input_cloud;
  std::string output_cloud;
  std::string ego_frame;
  std::string partner_frame;


  int watchdog_cloud;
  int watchdog_odom;
  int watchdog_vel;
  int watchdog_init;
  
  ros::NodeHandle nh;
  ros::Subscriber sub_cloud;
  ros::Publisher output_pub;
  tf::TransformListener tf;
  
  sensor_msgs::PointCloud2 cloud;
  XmlRpc::XmlRpcValue footprint_xml;

  void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);

};




limiter::limiter():
  nh("~"),
  tf_listener()
	
{
	nh.param("input_cloud", input_cloud, std::string("input_cloud"));
	nh.param("output_cloud", output_cloud, std::string("footprint_cloud"));

	nh.param("ego_frame", ego_frame, std::string("/v1/base_link"));
	nh.param("partner_frame", partner_frame, std::string("/v2/base_link"));

	

	sub_cloud = nh.subscribe(input_cloud, 2, &limiter::cloudCallback, this);
	// output_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > (output_cloud.c_str(), 10);

	output_pub = nh.advertise<sensor_msgs::PointCloud> (output_cloud.c_str(), 10);
	
	
	nh.getParam("footprint", footprint_xml);
	
	if(footprint_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xml.size() < 3)
	  {
		ROS_FATAL("Illegal footprint");
		throw std::runtime_error("Illegal footprint");
	}
	
}
	
	
	limiter::~limiter()
{
}



void limiter::cloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  sensor_msgs::PointCloud cloud1;
  cloud1 = *msg;
  sensor_msgs::PointCloud2 cloud2;
  sensor_msgs::PointCloud2 cloud3;

  //sensor_msgs::convertPointCloudToPointCloud2 ( *msg, cloud2 );
  sensor_msgs::convertPointCloudToPointCloud2 ( *msg, cloud3 );
  pcl::PointCloud<pcl::PointXYZ>::Ptr footprint_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Affine3d eigen_transform;
  tf::StampedTransform transform;


  //Transform from partner_frame to ego_frame  (the points of the partner frame to ego frame
  try{
    tf_listener.waitForTransform( ego_frame, partner_frame, msg->header.stamp, ros::Duration(0.5));
    tf_listener.lookupTransform    ( ego_frame, partner_frame, msg->header.stamp, transform);
  }	
  catch (tf::TransformException ex){
    ROS_ERROR("The error is:  %s",ex.what());
    return;
  }


  //pcl::PointIndices indices;
  //indices.indices.push_back(0);
  //indices.indices.push_back(2);
  


  //ROS_ERROR("Debug1 ; %d",  (int)msg->points.size() );  
  //geometry_msgs::Point32 point;
  double prev_x, prev_y ;
  double first_x, first_y ;
  for(int i = 0; i < (int)footprint_xml.size(); i ++){
    if (i==0){
      footprint_cloud->push_back (pcl::PointXYZ((double)footprint_xml[i][0], (double)footprint_xml[i][1], 0.0));
      first_x = (double)footprint_xml[i][0];
      first_y = (double)footprint_xml[i][1];
    }
    else{
      footprint_cloud->push_back (pcl::PointXYZ((double)footprint_xml[i][0], (double)footprint_xml[i][1], 0.0));
      footprint_cloud->push_back (pcl::PointXYZ( ((double)footprint_xml[i][0]+prev_x)/2 , ((double)footprint_xml[i][1]+prev_y)/2, 0.0));
    }
    prev_x = (double)footprint_xml[i][0];
    prev_y = (double)footprint_xml[i][1];  
  }
  
  footprint_cloud->push_back (pcl::PointXYZ( (first_x +prev_x)/2, (first_y + prev_y)/2, 0.0 ));


  
  
  
  // Put in the Eigen::Affine3d 
  tf::transformTFToEigen (transform, eigen_transform);

  
  // Apply the transform
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tmp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud( *footprint_cloud, *pcl_tmp, eigen_transform);
  footprint_cloud = pcl_tmp;
  
  //TODO
  //HAVE TO ADD THE TRANSFORMED POINTS TO THE original cloud:
  //    footprint_cloud + pcl_cloud3

  //pcl::concatenatePointCloud(*footprint_cloud, *pcl_cloud3, *footprint_cloud);
  
	
  
  footprint_cloud->header.frame_id = ego_frame ;


  for (sensor_msgs::PointCloud2ConstIterator<float> it(cloud3, "x"); it != it.end(); ++it) 
    footprint_cloud->push_back (pcl::PointXYZ( it[0], it[1], it[2]));

  
  
  pcl::toROSMsg( *footprint_cloud, cloud2 );
  //pcl::concatenatePointCloud(cloud2, cloud3, cloud2);

  /*
    x y z
    x y z index

  //std::vector<sensor_msgs::PointField> fields2;
  //std::vector<int> fields2_sizes;

  for (size_t j = 0; j < cloud2.fields.size (); ++j)
      std::cerr << " " << cloud2.fields[j].name ;
  std::cerr << "\n" ;

  for (size_t j = 0; j < cloud3.fields.size (); ++j)
      std::cerr << " " << cloud3.fields[j].name ;
  std::cerr << "\n\n" ;
  */  
  
  sensor_msgs::convertPointCloud2ToPointCloud ( cloud2, cloud1 );
  output_pub.publish( cloud1 );
  //output_pub.publish( my );

  //ROS_ERROR("Debug2  %d   %d", (int)my->size(), (int)footprint_xml.size() );  
  //watchdog_cloud = watchdog_init;




  
}




void limiter::spin()
{

  ros::spin();
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "safety_limiter");
	
	//ros::spin();
	limiter lim;
	lim.spin();

	return 0;
}

