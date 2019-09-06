/**
 * @file
 * @author  Jonas Furrer <furrer@atr.jp>
 * @version 1.0
 *
 * @brief TODO
 */

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

class Pc2Transformer
{
public:
    Pc2Transformer();

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in);

    ros::NodeHandle nh_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tf_listener_;
    ros::Subscriber sub_scan_;
    ros::Publisher pub_cloud_;
    std::string target_frame_name_;
    double range_cutoff_;
};

Pc2Transformer::Pc2Transformer():
    nh_("~")
{
    nh_.param("target_frame", target_frame_name_, std::string("/base_link"));
    nh_.param("range_cutoff", range_cutoff_, 10.0);
    sub_scan_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Pc2Transformer::scanCallback, this);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
}

void Pc2Transformer::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
    if (!tf_listener_.waitForTransform(
            scan_in->header.frame_id,
            target_frame_name_,
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
            ros::Duration(1.0)))
    {
        return;
    }

    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud(target_frame_name_, *scan_in, cloud, tf_listener_, range_cutoff_);

    pub_cloud_.publish(cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc2_transformer");
    Pc2Transformer transformer;
    ros::spin();
    return 0;
}
