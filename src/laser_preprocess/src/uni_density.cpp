#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

class unidensityNode
{
public:
	unidensityNode();
private:
	ros::NodeHandle n;
	ros::Publisher scan_pub;
	ros::Subscriber scan_sub;

	std::string from;
	std::string to;
	double cull_dist;
	double mixed_dist;

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_org);
};
 
void unidensityNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_org)
{
	sensor_msgs::LaserScan scan = *scan_org;
	float x[scan.ranges.size()], y[scan.ranges.size()];
	float xavg, yavg;
	int navg;
	int j = -1;

	for(int i = 0; i < scan.ranges.size(); i++)
	{
		if( scan.ranges[i] <= scan.range_min && scan.range_max <= scan.ranges[i] )
			continue;
		x[i] = scan.ranges[i] * cosf( scan.angle_min + scan.angle_increment * i );
		y[i] = scan.ranges[i] * sinf( scan.angle_min + scan.angle_increment * i );
		if( j < 0 )
		{
			xavg = x[i];
			yavg = y[i];
			navg = 1;
			j = i;
			continue;
		}

		if( powf(x[i] - x[j], 2) + powf(y[i] - y[j], 2) > powf(cull_dist, 2) || 
				i == scan.ranges.size() - 1)
		{
			xavg /= navg;
			yavg /= navg;
			for(int k = j; k < i; k ++)
			{
				scan.ranges[k] = 0;
			}
			if( navg > 1 || 
					( j > 0 && fabs( scan.ranges[j] - scan.ranges[j - 1] ) < mixed_dist ) ||
					fabs( scan.ranges[i - 1] - scan.ranges[i] ) < mixed_dist ||
					scan.ranges[j] * sinf( scan.angle_increment ) > mixed_dist)
			{
				int ang;
				ang = lroundf( ( atan2f( yavg, xavg ) - scan.angle_min ) / scan.angle_increment );
				scan.ranges[ang] = scan_org->ranges[ang];
			}
			xavg = yavg = navg = 0;
			j = i;
		}
		xavg += x[i];
		yavg += y[i];
		navg ++;
	}
	scan_pub.publish(scan);
}

unidensityNode::unidensityNode():
	n("~"),
	from("scan_orig"),
	to("scan"),
	cull_dist(0.1),
	mixed_dist(0.4)
{
	n.param( "scan", from, from );
	n.param( "uscan", to, to );
	n.param( "cull_dist", cull_dist, cull_dist );
	n.param( "mixed_dist", mixed_dist, mixed_dist );

	//std::cerr << from << " > " << to << " (" << cull_dist << "," << mixed_dist << ")" << std::endl;
	scan_pub = n.advertise<sensor_msgs::LaserScan>(to, 200);
	scan_sub = n.subscribe(from, 1, &unidensityNode::scanCallback, this);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uni_density");

	unidensityNode unidensity;
	ros::spin();

	return 0;
}
 
