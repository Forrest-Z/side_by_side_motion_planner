#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

using namespace laser_assembler;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "periodic_assemble_cloud2s");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	ros::service::waitForService("assemble_scans2");
	ros::ServiceClient client = n.serviceClient<AssembleScans2>("assemble_scans2");
	ros::Publisher pub_cloud = n.advertise<sensor_msgs::PointCloud2>("clouds", 10);
	tf::TransformListener tf;

	double hz;
	double duration;
	double delay;
	std::string tf_prefix;
	pn.param("hz", hz, double(10));
	pn.param("duration", duration, double(0.3));
	pn.param("delay", delay, double(0.05));
	pn.param("tf_prefix", tf_prefix, std::string(""));

	ros::Rate loop_rate(hz);
	while(ros::ok())
	{
		AssembleScans2 srv;
		ros::Time now = ros::Time::now() - ros::Duration(delay);
		srv.request.end   = now;
		srv.request.begin = now - ros::Duration(duration);
		if(client.call(srv))
		{
			sensor_msgs::PointCloud2 _cloud;
			try
			{
				tf.waitForTransform(tf_prefix+"/base_link", srv.response.cloud.header.frame_id, srv.request.end, ros::Duration(0.2));
				pcl_ros::transformPointCloud(tf_prefix+"/base_link", srv.response.cloud, _cloud, tf);
				pub_cloud.publish(_cloud);
			}
			catch (tf::TransformException &e)
			{
				ROS_WARN("TF exception: %s", e.what());
			}
		}
		else
		{
			ROS_WARN("Failed to call assemble_scans2");
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

