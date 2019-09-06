#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"
#include "laser_geometry/laser_geometry.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "eigen_conversions/eigen_msg.h"
#include "tf/transform_listener.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/pca.h"
#include "pcl/common/geometry.h"
#include "pcl/common/common.h"

class unidensityNode
{
public:
	unidensityNode();
private:
	ros::NodeHandle n;
	ros::Publisher scan_pub;
	ros::Publisher cloud_pub;
	ros::Publisher pca_pub;
	ros::Subscriber scan_sub;
    laser_geometry::LaserProjection projector;
    tf::TransformListener tf_listener;

	std::string from;
	std::string to;
	float interval;
	float secondAxisDef;
	enum
	{
		MEAN,
		MEDIAN
	} mode;

	void cloud2scan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			const sensor_msgs::LaserScan& scan_param);
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_org);
};
 
void unidensityNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::PointCloud2 cloud2;
    projector.transformLaserScanToPointCloud(scan->header.frame_id, 
			*scan, cloud2, tf_listener);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(cloud2, *cloud);

	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloud);
	Eigen::Matrix3f vecs = pca.getEigenVectors();
	Eigen::Vector3f egns = pca.getEigenValues();
	Eigen::Vector3f variance = egns / (float)cloud->points.size();
	Eigen::Vector4f center = pca.getMean();
	Eigen::Quaternionf q;
	for(int i = 0; i < 2; i ++) variance[i] = sqrtf(variance[i]);
	variance[1] *= secondAxisDef;

	geometry_msgs::PoseArray pa;
	geometry_msgs::Pose pose;
	pa.header = scan->header;
	tf::pointEigenToMsg(center.head<3>().cast<double>(), pose.position);
	
	q.setFromTwoVectors(Eigen::Vector3f(1.0, 0.0, 0.0), vecs.col(0));
	tf::quaternionEigenToMsg(q.cast<double>(), pose.orientation);
	pa.poses.push_back(pose);
	
	q.setFromTwoVectors(Eigen::Vector3f(1.0, 0.0, 0.0), vecs.col(1));
	tf::quaternionEigenToMsg(q.cast<double>(), pose.orientation);
	pa.poses.push_back(pose);
	pca_pub.publish(pa);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pj_uni(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_pj_uni = *cloud;
	cloud_pj_uni->points.clear();
	pcl::PointXYZ *last = &cloud->points[0];
	int ilast = 0;
	Eigen::Vector3f mean(0.0, 0.0, 0.0);
	int num = 0;
	for(size_t i = 0; i < cloud->points.size(); i ++)
	{
		auto &p = cloud->points[i];
		pca.project(p, p);
		if(fabs(variance[0]) > 0.0) p.x /= variance[0];
		if(fabs(variance[1]) > 0.0) p.y /= variance[1];

		if((last->getVector3fMap() - p.getVector3fMap()).norm() > interval)
		{
			if(i - ilast > 1 || 
					last->getVector3fMap().cross(p.getVector3fMap()).norm() / last->getVector3fMap().norm() >= 0.5 * interval)
			{
				pcl::PointXYZ pj_r;
				mean /= (float)num;

				switch(mode)
				{
				case MEAN:
					pj_r = pcl::PointXYZ(mean[0], mean[1], mean[2]);
					break;
				case MEDIAN:
					pj_r = cloud->points[(i + ilast)/2];
					break;
				}

				if(fabs(variance[0]) > 0.0) pj_r.x *= variance[0];
				if(fabs(variance[1]) > 0.0) pj_r.y *= variance[1];
				pca.reconstruct(pj_r, pj_r);
				cloud_pj_uni->points.push_back(pj_r);
			}
			mean = Eigen::Vector3f(0.0, 0.0, 0.0);
			num = 0;
			last = &p;
			ilast = i;
		}
		mean += p.getVector3fMap();
		num ++;
	}
	cloud_pj_uni->width = cloud_pj_uni->points.size();
    
	sensor_msgs::PointCloud2 cloud_out;
	pcl::toROSMsg(*cloud_pj_uni, cloud_out);
	cloud_out.header = scan->header;
	cloud_pub.publish(cloud_out);

	cloud2scan(cloud_pj_uni, *scan);
}

void unidensityNode::cloud2scan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		const sensor_msgs::LaserScan& scan_param)
{
	sensor_msgs::LaserScan scan = scan_param;
	for(auto &p: scan.ranges) p = 0;
	int iang_max = scan.ranges.size();

	for(auto &p: cloud->points)
	{
		float ang = atan2f(p.y, p.x);
		float dist = p.getVector3fMap().norm();
		int iang = lroundf((ang - scan.angle_min) / scan.angle_increment);
		if(0 <= iang && iang < iang_max) scan.ranges[iang] = dist;
	}
	scan_pub.publish(scan);
}

unidensityNode::unidensityNode():
	n("~")
{
	double resolution;
	double deformation;
	n.param( "scan", from, std::string("scan_orig") );
	n.param( "uscan", to, std::string("scan") );
	n.param( "resolution", resolution, 10.0 );
	n.param( "second_axis_deformation", deformation, 2.0 );
	interval = 1.0 / resolution;
	secondAxisDef = 1.0 / deformation;
	mode = MEDIAN;

	//std::cerr << from << " > " << to << " (" << cull_dist << "," << mixed_dist << ")" << std::endl;
	scan_pub = n.advertise<sensor_msgs::LaserScan>(to, 2);
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 2);
	pca_pub = n.advertise<geometry_msgs::PoseArray>("pca", 2);
	scan_sub = n.subscribe(from, 1, &unidensityNode::scanCallback, this);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uni_density2d");

	unidensityNode unidensity;
	ros::spin();

	return 0;
}
 
