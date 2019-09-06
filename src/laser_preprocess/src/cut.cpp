#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

class cutNode
{
public:
	cutNode();
private:
	ros::NodeHandle n;
	ros::Publisher scan_pub;
	ros::Subscriber scan_sub;

	std::string from;
	std::string to;
	int start;
	int end;

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_org);
};
 
void cutNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_org)
{
	sensor_msgs::LaserScan scan = *scan_org;

	for(int i = 0; i < scan.ranges.size(); i++)
	{
		if( start <= i && i <= end ) scan.ranges[i] = 0;
	}
	scan_pub.publish(scan);
}

cutNode::cutNode():
	n("~"),
	from("scan_orig"),
	to("scan"),
	end(0),
	start(0)
{
	n.param( "scan", from, from );
	n.param( "uscan", to, to );
	n.param( "start", start, start );
	n.param( "end", end, end );

	//std::cerr << from << " > " << to << " (" << cull_dist << "," << mixed_dist << ")" << std::endl;
	scan_pub = n.advertise<sensor_msgs::LaserScan>(to, 200);
	scan_sub = n.subscribe(from, 1, &cutNode::scanCallback, this);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cut");

	cutNode cut;
	ros::spin();

	return 0;
}
 
