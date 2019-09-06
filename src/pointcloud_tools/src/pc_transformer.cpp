#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

class Transformer 
{

public: 
    Transformer();

private:
    ros::NodeHandle             m_nh;
    tf::TransformListener       m_tf_listener;
    message_filters::Subscriber<sensor_msgs::PointCloud2>*
                                m_cloud_sub;
    tf::MessageFilter<sensor_msgs::PointCloud2>*
                                m_tfcloud_sub;
    ros::Publisher              m_cloud_pub;
    ros::Publisher              m_point_pub;

    std::string                 m_out_frame;

    float                       m_downsample_leafsize;

    void callback(
        const sensor_msgs::PointCloud2::ConstPtr&        msg);

    void get_eigen_transform(
        const std::string&                               in_frame,
        const ros::Time&                                 time,
        Eigen::Affine3d&                                 eigen_transform);
};

Transformer::Transformer():   
    m_nh("~"), 
    m_tf_listener()
{
    std::string topic;
    m_nh.param<std::string>("out_frame", m_out_frame, "/map");
    m_nh.param("downsample_leafsize", m_downsample_leafsize, 0.f);

    m_cloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh, "in_cloud", 5);
    m_tfcloud_sub = new tf::MessageFilter<sensor_msgs::PointCloud2> 
        (*m_cloud_sub, m_tf_listener, m_out_frame, 5);
    m_tfcloud_sub->registerCallback(boost::bind(&Transformer::callback, this, _1));

    m_cloud_pub = m_nh.advertise< sensor_msgs::PointCloud2 >( 
        "out_cloud", 5);
    m_point_pub = m_nh.advertise< geometry_msgs::PointStamped >( 
        "center_point", 5);
}

void Transformer::callback(
    const sensor_msgs::PointCloud2::ConstPtr&        msg)

{
    Eigen::Affine3d eigen_transform;
    try {
        get_eigen_transform(msg->header.frame_id, msg->header.stamp, eigen_transform);
    }
    catch (const tf::TransformException& e){
        ROS_ERROR("%s", e.what());
        return;
    }

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    fromROSMsg (*msg, *cloud);

    if (m_downsample_leafsize) {
        // Downsample
        pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr temp (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
        pcl::VoxelGrid<velodyne_pointcloud::PointXYZIR> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (m_downsample_leafsize, m_downsample_leafsize, m_downsample_leafsize);
        sor.filter (*temp);
        cloud = temp;
    }

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr out_cloud (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::transformPointCloud(*cloud, *out_cloud, eigen_transform);
    
    pcl::PointXYZ center_point(0,0,0);
    pcl::PointXYZ out_point = pcl::transformPoint(center_point, eigen_transform);


    out_cloud->header.stamp = cloud->header.stamp;
    out_cloud->header.frame_id = m_out_frame;
    m_cloud_pub.publish(out_cloud);

    geometry_msgs::PointStamped pt_msg;
    pt_msg.header.stamp = msg->header.stamp;
    pt_msg.header.frame_id = m_out_frame;
    pt_msg.point.x = out_point.x;
    pt_msg.point.y = out_point.y;
    pt_msg.point.z = out_point.z;
    m_point_pub.publish(pt_msg);
}

void Transformer::get_eigen_transform(
    const std::string&                               in_frame,
    const ros::Time&                                 time,
    Eigen::Affine3d&                                 eigen_transform)
{
    tf::StampedTransform transform;
    m_tf_listener.lookupTransform( m_out_frame, in_frame, time, transform);
    tf::transformTFToEigen(transform, eigen_transform);
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "pc_transformer");
    Transformer transformer;
    ros::spin();
    return 0;
}
