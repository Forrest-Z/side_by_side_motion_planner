#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

using namespace laser_assembler;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "periodic_assemble_scans");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	double hz;
	double duration;
	double delay;
	std::string tf_prefix,service_name;
	pn.param("hz", hz, double(10));
	pn.param("duration", duration, double(0.3));
	pn.param("delay", delay, double(0.05));
	pn.param("tf_prefix", tf_prefix, std::string(""));
	pn.param("assemble_scans",service_name,std::string("assemble_scans"));

	ros::service::waitForService(service_name);
	ros::ServiceClient client = n.serviceClient<AssembleScans>(service_name);
	ros::Publisher pub_cloud = n.advertise<sensor_msgs::PointCloud>("cloud", 10);
	tf::TransformListener tf;

	ros::Rate loop_rate(hz);
	while(ros::ok())
	{
		AssembleScans srv;
		ros::Time now = ros::Time::now() - ros::Duration(delay);
		srv.request.end   = now;
		srv.request.begin = now - ros::Duration(duration);
		if(client.call(srv))
		{
			sensor_msgs::PointCloud _cloud;
			try
			{
				tf.waitForTransform(tf_prefix+"/base_link", srv.response.cloud.header.frame_id, srv.request.end, ros::Duration(0.2));
				tf.transformPointCloud(tf_prefix+"/base_link", srv.response.cloud, _cloud);
				pub_cloud.publish(_cloud);
			}
			catch (tf::TransformException &e)
			{
				ROS_WARN("TF exception: %s", e.what());
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

