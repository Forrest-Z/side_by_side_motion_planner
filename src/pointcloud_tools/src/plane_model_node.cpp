#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <eigen3/Eigen/Core>

class PlaneModel 
{
public:
    PlaneModel();

private:
    ros::NodeHandle             m_nh;
    ros::Subscriber             m_sub;
    ros::Publisher              m_pub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr 
                                m_cloud;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr 
                                m_model;
    pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr
                                m_ransac;
    pcl::ModelCoefficients      m_coeffs;

    void callback(
        const sensor_msgs::PointCloud2::ConstPtr    msg);
};

PlaneModel::PlaneModel():
    m_nh("~"),
    m_cloud(new pcl::PointCloud<pcl::PointXYZ>),
    m_model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(m_cloud)),
    m_ransac(new pcl::RandomSampleConsensus<pcl::PointXYZ>(m_model, 0.05))
{
    m_sub = m_nh.subscribe("in_cloud", 10, &PlaneModel::callback, this);
    m_pub = m_nh.advertise< pcl_msgs::ModelCoefficients >("model_coefficients", 10);
}

void PlaneModel::callback(
    const sensor_msgs::PointCloud2::ConstPtr    msg)
{
    fromROSMsg(*msg, *m_cloud);

    if (m_cloud->points.size() == 0) return;

    m_model->setInputCloud(m_cloud);
    m_ransac->computeModel();

    Eigen::VectorXf coeffs;
    m_ransac->getModelCoefficients(coeffs);
    m_coeffs.values.resize(coeffs.size());
    Eigen::VectorXf::Map(&m_coeffs.values[0], coeffs.size()) = coeffs;

    m_coeffs.header = m_cloud->header;

    pcl_msgs::ModelCoefficients coeffs_msg;
    pcl_conversions::fromPCL(m_coeffs, coeffs_msg);
    m_pub.publish(coeffs_msg);
}


int main (int argc, char **argv)
{
    // initalize
    ros::init(argc, argv, "plane_model_node");
    PlaneModel pm;
    ros::spin();
    return 0;
}
