#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_types.h"
#include "tf/transform_listener.h"

#include <deque>

class mergepcNode
{
public:
	mergepcNode();
private:
	ros::NodeHandle n;
	ros::Publisher pubCloud;
	ros::Subscriber subCloud;
	tf::TransformListener tfl;

	int nCache;
	int nDelay;
	std::string frame_id;
	std::map<std::string, std::deque<sensor_msgs::PointCloud2>> cache;
	std::vector<std::string> frames;

	void cbCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);
};
 
void mergepcNode::cbCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
	if(cache[(*cloud).header.frame_id].size() == 0)
	{
		frames.push_back(cloud->header.frame_id);
	}
	cache[(*cloud).header.frame_id].push_front(*cloud);
	if(cache[(*cloud).header.frame_id].size() >= (size_t)nCache)
		cache[(*cloud).header.frame_id].pop_back();

	if(frames.size() <= 1) return;
	if((*cloud).header.frame_id.compare(frames[0]) != 0) return;

	ros::Time t_begin = (*(cache[frames[0]].begin()+nDelay)).header.stamp;
	ros::Time t_end = (*(cache[frames[0]].begin()+1+nDelay)).header.stamp;
	ros::Duration dt = t_begin - t_end;
	
	pcl::PointCloud<pcl::PointXYZI> pc_merged;
	pc_merged.header.frame_id = frame_id;
	pc_merged.header.stamp = t_begin.toNSec();
	for(auto &frame: frames)
	{
		ros::Duration t_near(dt.toSec() / 2.0);
		sensor_msgs::PointCloud2 *pc_near(nullptr);
		for(auto it = cache[frame].begin(); it != cache[frame].end(); it ++)
		{
			auto &pc = *it;
			if(pc.header.stamp == ros::Time(0)) continue;
			if(fabs(t_near.toSec()) > fabs((pc.header.stamp - t_begin).toSec()))
			{
				t_near = pc.header.stamp - t_begin;
				pc_near = &pc;
			}
		}
		if(pc_near != nullptr)
		{
			sensor_msgs::PointCloud2 pc_g;
			pcl::PointCloud<pcl::PointXYZI> pc_pcl;
			try
			{
				tfl.waitForTransform(frame_id, pc_near->header.frame_id, 
						pc_near->header.stamp, ros::Duration(0.05));
				pcl_ros::transformPointCloud(frame_id, *pc_near, pc_g, tfl);
				pcl::fromROSMsg(pc_g, pc_pcl);
				pc_merged += pc_pcl;
			}
			catch(tf::TransformException &e)
			{
				ROS_WARN("%s", e.what());
			}
		}
	}
	sensor_msgs::PointCloud2 pc_output;
	pcl::toROSMsg(pc_merged, pc_output);
	pc_output.header.stamp = t_begin;
	pubCloud.publish(pc_output);
}

mergepcNode::mergepcNode():
	n("~")
{
	n.param("cache_num", nCache, 5);
	n.param("delay_step", nDelay, 2);
	n.param("frame_id", frame_id, std::string("base_link"));

	pubCloud = n.advertise<sensor_msgs::PointCloud2>("clouds", 1);
	subCloud = n.subscribe("cloud", 2, &mergepcNode::cbCloud, this);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "merge_clouds");

	mergepcNode mergepc;
	ros::spin();

	return 0;
}
 
