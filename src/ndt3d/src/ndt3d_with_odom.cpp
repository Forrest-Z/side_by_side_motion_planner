/*
  Normal Distributions Transform test program.

  2005/4/24 tku
  2016/8/19 akai (modify: add odometry and pose fusion)
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <GL/glut.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/Marker.h>
#include <ndt3d/NDTResult.h>
#include <Eigen/Dense>

#include "ndt.h"
#include "algebra.h"

#define SENIOR_CAR
//#define RECORD_TRAJECTORY
//#define RECORD_NDMAP
//#define SAVE_VELODYNE_PCD

#define ODOM_BUF_SIZE 500

NDMapPtr NDmap;
NDPtr NDs;
int NDs_num;

char scan_file_base_name[100];
// row scan
Point scan_points[130000];
double scan_points_i[130000];
double scan_points_e[130000];
int scan_points_num;
double scan_points_weight[130000];
double scan_points_totalweight;

// flags
int is_first_time = 1;
int is_map_exist = 0;

// nd map
Point map_points[130000];
double map_points_i[130000];
int mapping_points_num;
ros::Publisher map_pub;

int scan_num;
int layer_select = LAYER_NUM - 1;
Posture prev_pose, prev_pose2;

//params
double g_map_center_x, g_map_center_y, g_map_center_z;
double g_map_rotation;
int g_map_x, g_map_y, g_map_z;
double g_map_cellsize;
char g_ndmap_name[500];
int g_use_gnss;
int g_map_update = 1;
double g_ini_x, g_ini_y, g_ini_z, g_ini_roll, g_ini_pitch, g_ini_yaw;

// prototype declaration
void print_matrix3d(double mat[3][3]);
void matrix_test(void);
void print_matrix6d(double mat[6][6]);

// newly added variables by Akai
typedef struct
{
	double x, y, z;
	double roll, pitch, yaw;
} pose_t;

typedef struct
{
	pose_t p;
	double time;
	bool is_data;
	double varxx, varyy, vartt, varxy, varyt, vartx;
} stamped_pose_t;

// voxel grid filter
double voxel_leaf_size = 1.0;
Point filtered_scan_points[130000];
double filtered_scan_points_i[130000];
double filtered_scan_points_e[130000];
int filtered_scan_points_num;
double filtered_scan_points_weight[130000];
double filtered_scan_points_totalweight;
ros::Publisher filtered_scan_pub;

// odometry for fusion
bool is_odom_initialized = false;
int new_odom_num, odom_interval;
nav_msgs::Odometry previous_odom;
pose_t vehicle_pose;
geometry_msgs::PoseWithCovarianceStamped vehicle_pose_msg;
ros::Publisher vehicle_pose_pub, error_elipse_pub, ndt_error_elipse_pub, ndt_current_velodyne_pose_pub, ndt_current_pose_with_cov_pub, filtered_current_pose_pub, velodyne_pose_pub, ndt_result_pub;
std::vector<stamped_pose_t> stamped_vehicle_poses;
double ndt_error, hessian[6][6], ndt_noise[6][6];
// new parameters
double odom_noise1, odom_noise2, odom_noise3, odom_noise4, odom_noise5, odom_noise6;
double ndt_noise_level, mahalanobis_dist_threshold;
double euclid_dist_threshold, yaw_andle_threshold;
double min_trace;
bool use_odometry, use_fusion, use_mahalanobis_dist, is_flat_surface, save_map, add_map_points, is_slam_mode, use_voxel_grid_filter;
std::string odom_topic_name;
bool debug_flag = true;
tf::TransformListener *tf_listener;
Eigen::VectorXd velodyne_point_in_base_link_frame(4), base_link_point_in_velodyne_frame(4);
bool is_tf_initialized = false;
bool do_global_reset = true;
std::string base_frame_id,velodyne_frame_id,velodyne_topic, ndt_frame_id, filtered_ndt_frame_id;



double pose_mod(Posture *pose)
{
	while (pose->theta  < -M_PI)	pose->theta  += 2.0 * M_PI;
	while (pose->theta  >  M_PI)	pose->theta  -= 2.0 * M_PI;
	while (pose->theta2 < -M_PI)	pose->theta2 += 2.0 * M_PI;
	while (pose->theta2 >  M_PI)	pose->theta2 -= 2.0 * M_PI;
	while (pose->theta3 < -M_PI)	pose->theta3 += 2.0 * M_PI;
	while (pose->theta3 >  M_PI)	pose->theta3 -= 2.0 * M_PI;
}

double nrand(double n)
{
	double r;
	r = n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX);
	return r;
}

void tf_init(void)
{
	tf_listener = new tf::TransformListener;
	tf::StampedTransform velodyne_2_base;
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		try
		{
			ros::Time now = ros::Time::now();

			tf_listener->waitForTransform(velodyne_frame_id, base_frame_id, now, ros::Duration(1));
			tf_listener->lookupTransform(velodyne_frame_id, base_frame_id, now, velodyne_2_base);
#
			break;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			loop_rate.sleep();
		}
	}

	tf::Point zero_point(0.0, 0.0, 0.0);
	tf::Point velodyne_point_in_base_frame_tf = velodyne_2_base.inverse() * zero_point;
	velodyne_point_in_base_link_frame(0) = velodyne_point_in_base_frame_tf.getX();
	velodyne_point_in_base_link_frame(1) = velodyne_point_in_base_frame_tf.getY();
	velodyne_point_in_base_link_frame(2) = velodyne_point_in_base_frame_tf.getZ();
	velodyne_point_in_base_link_frame(3) = 1.0;
	tf::Point base_link_point_in_velodyne_frame_tf = velodyne_2_base * zero_point;
	base_link_point_in_velodyne_frame(0) = base_link_point_in_velodyne_frame_tf.getX();
	base_link_point_in_velodyne_frame(1) = base_link_point_in_velodyne_frame_tf.getY();
	base_link_point_in_velodyne_frame(2) = base_link_point_in_velodyne_frame_tf.getZ();
	base_link_point_in_velodyne_frame(3) = 1.0;

	is_tf_initialized = true;
	fprintf(stderr, "tf is initialized\n");
}

bool is_miss_convergence(pose_t odom, Eigen::MatrixXd odom_cov, pose_t ndt, double d_th, double y_th, double m_th)
{
	if (is_first_time)
		return false;

	double dx, dy, dyaw;
	dx = odom.x - ndt.x;
	dy = odom.y - ndt.y;
	dyaw = odom.yaw - ndt.yaw;
	while (dyaw < -M_PI)  dyaw += 2.0 * M_PI;
	while (dyaw > M_PI)   dyaw -= 2.0 * M_PI;
	double dl = sqrt(dx * dx + dy * dy);
	if (debug_flag)
	{
		fprintf(stderr, "dl = %.3lf [m], dyaw = %.3lf [deg], ", dl, fabs(dyaw) * 180.0 / M_PI);
		fprintf(stderr, "d_th = %.3lf [m], y_th = %.3lf [deg]\n", d_th, y_th);
	}

	if (dl >= d_th || fabs(dyaw) >= y_th * M_PI / 180.0)
	{
		ROS_ERROR("reject scan matching result: estimated poses by odometry and ndt scan matching are far");
		return true;
	}

	if (use_mahalanobis_dist && use_fusion)
	{
		double m_dist;
		Eigen::VectorXd e(3);
		e(0) = odom.x - ndt.x;
		e(1) = odom.y - ndt.y;
		e(2) = odom.yaw - ndt.yaw;
		while (e(2) < -M_PI)  e(2) += 2.0 * M_PI;
		while (e(2) > M_PI)   e(2) -= 2.0 * M_PI;
//		m_dist = sqrt(e.transpose() * odom_cov.inverse() * e);
		Eigen::MatrixXd cov(3, 3);
		for (int j = 0; j < 3; j++)
		{
			for (int i = 0; i < 3; i++)	cov(i, j) = odom_cov(i, j);
		}
		Eigen::MatrixXd tmp(3, 1);
		tmp = odom_cov.inverse() * e;
		m_dist = sqrt(e(0) * tmp(0, 0) + e(1) * tmp(1, 0) + e(2) * tmp(2, 0));
		// odom_cov.determinant();
		if (debug_flag)	fprintf(stderr, "m_dist = %.3lf, m_th = %.3lf\n", m_dist, m_th);
		if (m_dist > m_th || isnan(m_dist) != 0)
		{
			ROS_ERROR("reject scan matching result: mahalanobis distance exceeded threshold");
			return true;
		}
	}

	return false;
}

void fusion_kalman(pose_t odom, Eigen::MatrixXd odom_cov, pose_t ndt, pose_t *fuse, std_msgs::Header header)
{
	// check hessian (is needed?)
	for (int i = 0; i < 6; i++)
	{
		if (hessian[i][i] < 0.001 || isinf(hessian[i][i]) != 0)
		{
			fuse->x = odom.x;
			fuse->y = odom.y;
			fuse->z = odom.z;
			fuse->roll = odom.roll;
			fuse->pitch = odom.pitch;
			fuse->yaw = odom.yaw;
			ROS_ERROR("unusable hessian: initial pose might be incorrect");
			return;
		}
	}
	// check Mahalanobis and Euclid distances from odometry to ndt scan matching result
	if (is_miss_convergence(odom, odom_cov, ndt, euclid_dist_threshold, yaw_andle_threshold, mahalanobis_dist_threshold))
	{
		fuse->x = odom.x;
		fuse->y = odom.y;
		fuse->z = odom.z;
		fuse->roll = odom.roll;
		fuse->pitch = odom.pitch;
		fuse->yaw = odom.yaw;
		if (debug_flag)	fprintf(stderr, "NDT scan matching might be failed\n");
		return;
	}

	// start fusion
	Eigen::VectorXd x(6), z(6), e(6), x2(6);
	Eigen::MatrixXd R_tmp(6, 6), R(6, 6), S(6, 6), K(6, 6), P2(6, 6);
	Eigen::MatrixXd P = Eigen::MatrixXd::Zero(6, 6);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
	// odometry pose
	x(0) = odom.x;
	x(1) = odom.y;
	x(2) = odom.z;
	x(3) = odom.roll;
	x(4) = odom.pitch;
	x(5) = odom.yaw;
	// NDT pose
	z(0) = ndt.x;
	z(1) = ndt.y;
	z(2) = ndt.z;
	z(3) = ndt.roll;
	z(4) = ndt.pitch;
	z(5) = ndt.yaw;
	// observation error
	e = z - x;
	while (e(3) < -M_PI)  e(3) += 2.0 * M_PI;
	while (e(3) > M_PI)   e(3) -= 2.0 * M_PI;
	while (e(4) < -M_PI)  e(4) += 2.0 * M_PI;
	while (e(4) > M_PI)   e(4) -= 2.0 * M_PI;
	while (e(5) < -M_PI)  e(5) += 2.0 * M_PI;
	while (e(5) > M_PI)   e(5) -= 2.0 * M_PI;
	// odometry pose covariance
	P(0, 0) = odom_cov(0, 0),  P(0, 1) = odom_cov(0, 1),  P(0, 2) = 0.0, P(0, 3) = 0.0, P(0, 4) = 0.0, P(0, 5) = odom_cov(0, 2);
	P(1, 0) = odom_cov(1, 0),  P(1, 1) = odom_cov(1, 1),  P(1, 2) = 0.0, P(1, 3) = 0.0, P(1, 4) = 0.0, P(1, 5) = odom_cov(1, 2);
	P(2, 0) = 0.0,             P(2, 1) = 0.0,             P(2, 2) = 0.0, P(2, 3) = 0.0, P(2, 4) = 0.0, P(2, 5) = 0.0;
	P(3, 0) = 0.0,             P(3, 1) = 0.0,             P(3, 2) = 0.0, P(3, 3) = 0.0, P(3, 4) = 0.0, P(3, 5) = 0.0;
	P(4, 0) = 0.0,             P(4, 1) = 0.0,             P(4, 2) = 0.0, P(4, 3) = 0.0, P(4, 4) = 0.0, P(4, 5) = 0.0;
	P(5, 0) = odom_cov(2, 0),  P(5, 1) = odom_cov(2, 1),  P(5, 2) = 0.0, P(5, 3) = 0.0, P(5, 4) = 0.0, P(5, 5) = odom_cov(2, 2);
	// NDT pose covariance
	for (int j = 0; j < 6; j++)
	{
		for (int i = 0; i < 6; i++)  R_tmp(i, j) = hessian[i][j];
	}
	int points_num;
	if (!use_voxel_grid_filter)
		points_num = scan_points_num;
	else
		points_num = filtered_scan_points_num;
	R = R_tmp.inverse();
	R *= ndt_noise_level;
	R *= (2.0 * ndt_error) / (double)(points_num - 3);
	for (int j = 0; j < 6; j++)
	{
		for (int i = 0; i < 6; i++)	ndt_noise[i][j] = R(i, j);
	}
	// observation error covariance
	S = R + P;
	// kalman gain
	K = P * (S.inverse());
	// update
	x2 = x + K * e;
	while (x2(3) < -M_PI)  x2(3) += 2.0 * M_PI;
	while (x2(3) > M_PI)   x2(3) -= 2.0 * M_PI;
	while (x2(4) < -M_PI)  x2(4) += 2.0 * M_PI;
	while (x2(4) > M_PI)   x2(4) -= 2.0 * M_PI;
	while (x2(5) < -M_PI)  x2(5) += 2.0 * M_PI;
	while (x2(5) > M_PI)   x2(5) -= 2.0 * M_PI;
	P2 = (I - K) * P;

	// overwrite
	fuse->x = x2(0);
	fuse->y = x2(1);
//	fuse->z = x2(2);
//	fuse->roll = x2(3);
//	fuse->pitch = x2(4);
	fuse->z = ndt.z;
	fuse->roll = ndt.roll;
	fuse->pitch = ndt.pitch;
	fuse->yaw = x2(5);

	// final check
	if (is_miss_convergence(odom, odom_cov, *fuse, euclid_dist_threshold / 6.0, yaw_andle_threshold / 5.0, mahalanobis_dist_threshold / 5.0))
	{
		// ndt result and odometry result are largely different?
		// ndt result would be failure...?
		// fuse_pose is overwritten by odometry result
		fuse->x = odom.x;
		fuse->y = odom.y;
		fuse->z = odom.z;
		fuse->roll = odom.roll;
		fuse->pitch = odom.pitch;
		fuse->yaw = odom.yaw;
		if (debug_flag)	fprintf(stderr, "Kalman filter fusion failure\n");
		return;
	}
	// update computation would be unstable when covariance is updated every time.
	// covariance size might be checked before update to prevent the computation failure.
	double trace = P(0, 0) + P(1, 1) + P(5, 5);
//	double trace = P2(0, 0) + P2(1, 1) + P2(5, 5);
	if (trace > min_trace)
	{
		for (int j = 0; j < 6; j++)
		{
			for (int i = 0; i < 6; i++)
				vehicle_pose_msg.pose.covariance[i * 6 + j] = P2(i, j);
		}
	}
}

void reset_odom_covariance(void)
{
	for (int i = 0; i < 36; i++)
		vehicle_pose_msg.pose.covariance[i] = 0.0;
	vehicle_pose_msg.pose.covariance[0 * 6 + 0] = 10.0;
	vehicle_pose_msg.pose.covariance[1 * 6 + 1] = 10.0;
	vehicle_pose_msg.pose.covariance[2 * 6 + 2] = 10.0;
	vehicle_pose_msg.pose.covariance[3 * 6 + 3] = 0.3;
	vehicle_pose_msg.pose.covariance[4 * 6 + 4] = 0.3;
	vehicle_pose_msg.pose.covariance[5 * 6 + 5] = 0.5;
}

void velodyne_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &msg)
{
	if (!is_odom_initialized)
	{
		ROS_ERROR("odometry is not initialized");
		return;
	}

	static int count = 0;
	std_msgs::Header header;
	pcl_conversions::fromPCL(msg->header, header);

	scan_points_totalweight = 0;
	filtered_scan_points_totalweight = 0;
	// record raw scan points
	{
		int j = 0;
		for (int i = 0; i < msg->points.size(); i++)
		{
			scan_points[j].x = msg->points[i].x;
			scan_points[j].y = msg->points[i].y;
			scan_points[j].z = msg->points[i].z;
			scan_points_i[j] = msg->points[i].intensity;
			double dist = scan_points[j].x * scan_points[j].x
				+ scan_points[j].y * scan_points[j].y
				+ scan_points[j].z * scan_points[j].z;
			if (dist < 3 * 3)	continue;
			scan_points_weight[j] = (dist) * (1.2 - exp(-1.0 * (-1.0 - scan_points[j].z) * (-1.0 - scan_points[j].z) / 4.0));
			scan_points_totalweight += scan_points_weight[j];
			j++;
			if (j > 130000)	break;
		}
		scan_points_num = j;
		mapping_points_num= j;
	}

	if (use_voxel_grid_filter)
	{
		// voxel grid filter
		pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(*msg));
		pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
		voxel_grid_filter.setInputCloud(scan_ptr);
		voxel_grid_filter.filter(*filtered_scan_ptr);
		sensor_msgs::PointCloud2 filtered_msg;
		filtered_msg.header.frame_id = header.frame_id;
		pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
		filtered_scan_pub.publish(filtered_msg);
		{
			int j = 0;
			for (int i = 0; i < filtered_scan_ptr->size(); i++)
			{
				filtered_scan_points[j].x = filtered_scan_ptr->points[i].x;
				filtered_scan_points[j].y = filtered_scan_ptr->points[i].y;
				filtered_scan_points[j].z = filtered_scan_ptr->points[i].z;
				filtered_scan_points_i[j] = filtered_scan_ptr->points[i].intensity;
				double dist = filtered_scan_points[j].x * filtered_scan_points[j].x
					+ filtered_scan_points[j].y * filtered_scan_points[j].y
					+ filtered_scan_points[j].z * filtered_scan_points[j].z;
				if (dist < 3 * 3)	continue;
//				filtered_scan_points_weight[j] = (dist) * (1.2 - exp(-1.0 * (-1.0 - filtered_scan_points[j].z) * (-1.0 - filtered_scan_points[j].z) / 4.0));
				filtered_scan_points_weight[j] = 1.0;
				filtered_scan_points_totalweight += filtered_scan_points_weight[j];
				j++;
				if (j > 130000)	break;
			}
			filtered_scan_points_num = j;
		}
	}

	// ndt scan matching (this pose represents velodyne position)
	Posture pose, bpose, initial_pose;
	static Posture key_pose;
	double e = 0.0, theta, x2, y2;
	double x_offset, y_offset, z_offset, theta_offset;
	double distance, diff;
	pose_t odom_pose;
	Eigen::MatrixXd odom_cov(3, 3);

	// calc offset from previous localization results
	x_offset = prev_pose.x - prev_pose2.x;
	y_offset = prev_pose.y - prev_pose2.y;
	z_offset = prev_pose.z - prev_pose2.z;
	theta_offset = prev_pose.theta3 - prev_pose2.theta3;
	if (theta_offset < -M_PI) theta_offset += 2.0 * M_PI;
	if (theta_offset >  M_PI) theta_offset -= 2.0 * M_PI;

	// calc initial position
	// time synchronization odometry and velodyne
	int pose_index = -1;
	double velodyne_time = header.stamp.toSec();
	for (int i = 0; i < ODOM_BUF_SIZE; i++)
	{
		if (velodyne_time > stamped_vehicle_poses[i].time)
		{
			if (i == 0 || i == ODOM_BUF_SIZE - 1)
			{
				pose_index = i;
			}
			else
			{
				double del1 = fabs(velodyne_time - stamped_vehicle_poses[i].time);
				double del2 = fabs(velodyne_time - stamped_vehicle_poses[i - 1].time);
				if (del1 < del2)
					pose_index = i;
				else
					pose_index = i - 1;
			}
			break;
		}
	}

	if (!use_odometry)
	{
		pose.x = prev_pose.x + x_offset;
		pose.y = prev_pose.y + y_offset;
		pose.z = prev_pose.z + z_offset;    
		pose.theta  = prev_pose.theta;
		pose.theta2 = prev_pose.theta2;
		pose.theta3 = prev_pose.theta3 + theta_offset;
		if (pose.theta3 < -M_PI) pose.theta3 += 2.0 * M_PI;
		if (pose.theta3 >  M_PI) pose.theta3 -= 2.0 * M_PI;
	}
	else if (pose_index < 0)
	{
		ROS_ERROR("no synchronized odometry data: computation time might be long");
		return;
	}
	else if (stamped_vehicle_poses[pose_index].is_data == false)
	{
		ROS_ERROR("no odometry data: is odometry data published?");
		return;
	}
	else
	{
		// vechicle pose is base_link, pose is velodyne
		// vechicle pose should be converted to velodyne position
		stamped_pose_t sp = stamped_vehicle_poses[pose_index];
		Eigen::MatrixXd base_2_vel(4, 4);
		double ca = cos(sp.p.roll);
		double sa = sin(sp.p.roll);
		double cb = cos(sp.p.pitch);
		double sb = sin(sp.p.pitch);
		double cg = cos(sp.p.yaw);
		double sg = sin(sp.p.yaw);
		base_2_vel(0, 0) = ca * cb * cg - sa * sg;
		base_2_vel(0, 1) = -ca * cb * sg - sa * cg;
		base_2_vel(0, 2) = ca * sb;
		base_2_vel(0, 3) = sp.p.x;
		base_2_vel(1, 0) = sa * cb * cg + ca * sg;
		base_2_vel(1, 1) = -sa * cb * sg + ca * cg;
		base_2_vel(1, 2) = sa * sb;
		base_2_vel(1, 3) = sp.p.y;
		base_2_vel(2, 0) = -sb * cg;
		base_2_vel(2, 1) = sb * sg;
		base_2_vel(2, 2) = cb;
		base_2_vel(2, 3) = sp.p.z;
		base_2_vel(3, 0) = 0.0;
		base_2_vel(3, 1) = 0.0;
		base_2_vel(3, 2) = 0.0;
		base_2_vel(3, 3) = 1.0;
		Eigen::VectorXd current_velodyne_point(4);
		current_velodyne_point = base_2_vel * velodyne_point_in_base_link_frame;
		// set initial pose (velodyne)
		pose.x = current_velodyne_point(0);
		pose.y = current_velodyne_point(1);
		pose.z = current_velodyne_point(2);
		pose.theta = prev_pose.theta;
		pose.theta2 = prev_pose.theta2;
		pose.theta3 = sp.p.yaw;
		// save initial odometry data for fusion (base_link)
		odom_pose.x = sp.p.x;
		odom_pose.y = sp.p.y;
		odom_pose.z = sp.p.z;
		odom_pose.roll = prev_pose.theta;
		odom_pose.pitch = prev_pose.theta2;
		odom_pose.yaw = sp.p.yaw;
		odom_cov(0, 0) = sp.varxx;
		odom_cov(0, 1) = sp.varxy;
		odom_cov(0, 2) = sp.vartx;
		odom_cov(1, 0) = sp.varxy;
		odom_cov(1, 1) = sp.varyy;
		odom_cov(1, 2) = sp.varyt;
		odom_cov(2, 0) = sp.vartx;
		odom_cov(2, 1) = sp.varyt;
		odom_cov(2, 2) = sp.vartt;
	}
	initial_pose = pose;

	// start ndt scan matching
	ndt3d::NDTResult ndt_result;
	std_msgs::Time start_time;
	start_time.data = ros::Time::now();
	if (!use_voxel_grid_filter)
		ndt_result.scan_points_num = scan_points_num;
	else
		ndt_result.scan_points_num = filtered_scan_points_num;
	for (layer_select = 2; layer_select >= 1; layer_select -= 1)
	{
		for (int j = 0; j < 50; j++)
		{
			bpose = pose;
			ndt_error = 0.0;
			if (!use_voxel_grid_filter)
				e = adjust3d(scan_points, scan_points_num, &pose, layer_select);
			else
				e = adjust3d(filtered_scan_points, filtered_scan_points_num, &pose, layer_select);
			pose_mod(&pose);
			if ((bpose.x - pose.x) * (bpose.x - pose.x) +
				(bpose.y - pose.y) * (bpose.y - pose.y) +
				(bpose.z - pose.z) * (bpose.z - pose.z) +
				3.0 * (bpose.theta - pose.theta) * (bpose.theta - pose.theta) +
				3.0 * (bpose.theta2 - pose.theta2) * (bpose.theta2 - pose.theta2) +
				3.0 * (bpose.theta3 - pose.theta3) * (bpose.theta3 - pose.theta3) < 0.00001)
			break;
		}

		// resetting initial position based on gps
		if (g_use_gnss)
		{
			bool do_pose_reset = false;
			if (!use_voxel_grid_filter)
			{
				if (layer_select == 1 && debug_flag)	fprintf(stderr, "sum of likelihood = %lf, points_num = %d\n", e, scan_points_num);
				if (layer_select == 1 && e < 1000.0)	do_pose_reset = true;
			}
			else
			{
				if (layer_select == 1 && debug_flag)	fprintf(stderr, "sum of likelihood = %lf, filtered_points_num = %d\n", e, filtered_scan_points_num);
				double th = (1000.0 - 100.0) / (130000.0 - 5000.0) * (double)filtered_scan_points_num;
				if (debug_flag)	fprintf(stderr, "th = %lf\n", th);
				if (layer_select == 1 && e < th)	do_pose_reset = true;
			}
			if (do_pose_reset)
//			if (do_pose_reset && do_global_reset)
			{
				tf::StampedTransform gps_tf_on_world;
				try
				{
					tf_listener->lookupTransform("/world", "/gps", ros::Time(0), gps_tf_on_world);
				}
				catch (tf::TransformException ex)
				{
					// no transform world to gps, initializing pose is not done
					ROS_ERROR("%s", ex.what());
					return;
				}
				// initialize pose
				// 本当はGPSレシーバからvelodyneとbase_linkフレームのtfを使ってちゃんと初期化しないといけない
				// （そもそもGPSの誤差が大きいのであまり気にしてない）
				pose.x = gps_tf_on_world.getOrigin().x();
				pose.y = gps_tf_on_world.getOrigin().y();
				pose.z = gps_tf_on_world.getOrigin().z() + nrand(1.0);
				tf::Quaternion q(gps_tf_on_world.getRotation().x(),
					gps_tf_on_world.getRotation().y(),
					gps_tf_on_world.getRotation().z(),
					gps_tf_on_world.getRotation().w());
				double roll, pitch, yaw;
				tf::Matrix3x3 m(q);
				m.getRPY(roll, pitch, yaw);
				pose.theta = roll;
				pose.theta2 = pitch + 0.22;
				pose.theta3 = yaw + nrand(0.1);
				prev_pose2 = prev_pose = pose;
				is_first_time = 1;
				vehicle_pose.x = pose.x;
				vehicle_pose.y = pose.y;
				vehicle_pose.z = pose.z;
				vehicle_pose.roll = pose.theta;
				vehicle_pose.pitch = pose.theta2;
				vehicle_pose.yaw = pose.theta3;
				reset_odom_covariance();
				if (debug_flag)
				{
					fprintf(stderr, "-----pose resetting by GPS-----\n");
					fprintf(stderr, "x = %.2lf, y = %.2lf, z = %.2lf [m]\n", vehicle_pose.x, vehicle_pose.y, vehicle_pose.z);
					fprintf(stderr, "roll = %.2lf, pitch = %.2lf, yaw = %.2lf [deg]\n", vehicle_pose.roll * 180.0 / M_PI, vehicle_pose.pitch * 180.0 / M_PI, vehicle_pose.yaw * 180.0 / M_PI);
					fprintf(stderr, "\n");
				}
				// 走行中にGPSのリセットが入ると危ないため，一度しか行わない（デモ用）
//				do_global_reset = false;
				return;
			}
		}

		// unti-distotion
		if (layer_select == 2 && 1)
		{
			double rate, angle, xrate, yrate, dx, dy, dtheta;
			double tempx, tempy;

			tempx = (pose.x - prev_pose.x);
			tempy = (pose.y - prev_pose.y);
			dx = tempx * cos(-prev_pose.theta3) - tempy * sin(-prev_pose.theta3);
			dy = tempx * sin(-prev_pose.theta3) + tempy * cos(-prev_pose.theta3);
			dtheta = pose.theta3 - prev_pose.theta3;
			if (dtheta < -M_PI)
				dtheta += 2.0 * M_PI;
			if (dtheta >  M_PI)
				dtheta -= 2.0 * M_PI;
			rate = dtheta / (double)scan_points_num;
			xrate = dx / (double)scan_points_num;
			yrate = dy / (double)scan_points_num;
			dx = -dx;
			dy = -dy;
			dtheta = -dtheta;
			for (int i = 0; i < scan_points_num; i++)
			{
				tempx = scan_points[i].x * cos(dtheta) - scan_points[i].y * sin(dtheta) + dx;
				tempy = scan_points[i].x * sin(dtheta) + scan_points[i].y * cos(dtheta) + dy;
				scan_points[i].x = tempx;
				scan_points[i].y = tempy;
				dtheta += rate;
				dx += xrate;
				dy += yrate;
			}
		}
	}
	std_msgs::Time end_time;
	end_time.data = ros::Time::now();
	ndt_result.computation_time = end_time.data.toSec() - start_time.data.toSec();
	ndt_result.header.stamp = header.stamp;
	ndt_result.matching_score = e;
	ndt_result.error_distance = ndt_error;
	ndt_result_pub.publish(ndt_result);
	// ndt scan matching end

	// code for taking machine learning data
	// ***** from here *****
	Posture tmp_pose = pose;
//	static FILE *fp_ndt_path;
//	if (!fp_ndt_path)	fp_ndt_path = fopen("/tmp/data/ndt_path.log", "w");
//	fprintf(fp_ndt_path, "%d %lf %lf %lf %lf %lf %lf %lf ", count, header.stamp.toSec(), pose.x, pose.y, pose.z, pose.theta, pose.theta2, pose.theta3);
//	for (int i = 0; i < 6; i++)
//	{
//		for (int j = 0; j < 6; j++)	fprintf(fp_ndt_path, "%lf ", ndt_noise[i][j]);
//	}
//	fprintf(fp_ndt_path, "\n");
	bool add_noise_ndt = false;
	ros::NodeHandle nh1;
	nh1.getParam("/ndt3d_with_odom/add_noise_ndt", add_noise_ndt);
	if (add_noise_ndt)
	{
		pose.x += nrand(1.0);
		pose.y += nrand(1.0);
//		pose.z += nrand(0.2);
//		pose.theta += nrand(0.2 * M_PI / 180.0);
//		pose.theta2 += nrand(0.2 * M_PI / 180.0);
		pose.theta3 += nrand(5.0 * M_PI / 180.0);
		pose_mod(&pose);
		add_noise_ndt = false;
		ROS_ERROR("add noise to ndt localization result");
		nh1.setParam("/ndt3d_with_odom/add_noise_ndt", add_noise_ndt);
//		static FILE *fp_add_noise;
//		if (!fp_add_noise)	fp_add_noise = fopen("/tmp/data/add_noise.log", "w");
//		fprintf(fp_add_noise, "%d %lf %lf %lf %lf %lf %lf %lf\n", count, header.stamp.toSec(), pose.x, pose.y, pose.z, pose.theta, pose.theta2, pose.theta3);
	}
	// ***** until here *****

	// publish current velodyne pose estimated by ndt scan matching
	geometry_msgs::PoseStamped ndt_current_velodyne_pose;
	ndt_current_velodyne_pose.header.frame_id = "/world";
	ndt_current_velodyne_pose.header.stamp = header.stamp;
	ndt_current_velodyne_pose.pose.position.x = pose.x;
	ndt_current_velodyne_pose.pose.position.y = pose.y;
	ndt_current_velodyne_pose.pose.position.z = pose.z;
	tf::Quaternion ndt_velodyne_q;
	ndt_velodyne_q.setRPY(pose.theta, pose.theta2, pose.theta3);
	geometry_msgs::Quaternion ndt_velodyne_q_msg;
	tf::quaternionTFToMsg(ndt_velodyne_q, ndt_velodyne_q_msg);
	ndt_current_velodyne_pose.pose.orientation = ndt_velodyne_q_msg;
	ndt_current_velodyne_pose_pub.publish(ndt_current_velodyne_pose);


	// 元に戻す！
//	pose = tmp_pose;


	// compute base_link pose esmtimated by ndt scan matching
	Eigen::MatrixXd vel_2_base(4, 4);
	double ca = cos(pose.theta);
	double sa = sin(pose.theta);
	double cb = cos(pose.theta2);
	double sb = sin(pose.theta2);
	double cg = cos(pose.theta3);
	double sg = sin(pose.theta3);
	vel_2_base(0, 0) = ca * cb * cg - sa * sg;
	vel_2_base(0, 1) = -ca * cb * sg - sa * cg;
	vel_2_base(0, 2) = ca * sb;
	vel_2_base(0, 3) = pose.x;
	vel_2_base(1, 0) = sa * cb * cg + ca * sg;
	vel_2_base(1, 1) = -sa * cb * sg + ca * cg;
	vel_2_base(1, 2) = sa * sb;
	vel_2_base(1, 3) = pose.y;
	vel_2_base(2, 0) = -sb * cg;
	vel_2_base(2, 1) = sb * sg;
	vel_2_base(2, 2) = cb;
	vel_2_base(2, 3) = pose.z;
	vel_2_base(3, 0) = 0.0;
	vel_2_base(3, 1) = 0.0;
	vel_2_base(3, 2) = 0.0;
	vel_2_base(3, 3) = 1.0;
	Eigen::VectorXd ndt_base_point(4);
	ndt_base_point = vel_2_base * base_link_point_in_velodyne_frame;

	// boadcast tf ndt_frame (base_link pose estimated by ndt scan matching)
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(ndt_base_point(0), ndt_base_point(1), ndt_base_point(2)));
	tf::Quaternion q;
	q.setRPY(pose.theta, pose.theta2, pose.theta3);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, header.stamp, "/world", ndt_frame_id));

	// estimate vehicle pose (base_link)
	pose_t ndt_pose, fuse_pose;
	ndt_pose.x = ndt_base_point(0);
	ndt_pose.y = ndt_base_point(1);
	ndt_pose.z = ndt_base_point(2);
	ndt_pose.roll = pose.theta;
	ndt_pose.pitch = pose.theta2;
	ndt_pose.yaw = pose.theta3;
	if (is_flat_surface)
	{
		ndt_pose.z = odom_pose.z = 0.0;
		ndt_pose.roll = odom_pose.roll = 0.0;
		ndt_pose.pitch = odom_pose.pitch = 0.0;
	}

	bool compute_cov = true;
	if (!use_odometry)
	{
		vehicle_pose = ndt_pose;
	}
	else
	{
		if (use_fusion)
		{
			// fuison odometry and NDT scan matching results
			pose_t fuse_pose;
			fusion_kalman(odom_pose, odom_cov, ndt_pose, &fuse_pose, header);
			vehicle_pose = fuse_pose;
			compute_cov = false;
		}
		else
		{
			// correct vehicle_pose by the NDT scan matching's result
			if (isnan(ndt_pose.x) != 0 || isnan(ndt_pose.y) != 0 || isnan(ndt_pose.z) != 0 ||
					isnan(ndt_pose.roll) != 0 || isnan(ndt_pose.pitch) != 0 || isnan(ndt_pose.yaw) != 0)
				vehicle_pose = odom_pose;
			else if (is_miss_convergence(odom_pose, odom_cov, ndt_pose, euclid_dist_threshold, yaw_andle_threshold, mahalanobis_dist_threshold))
				vehicle_pose = odom_pose;
			else
				vehicle_pose = ndt_pose;
			for (int i = 0; i < 36; i++)
				vehicle_pose_msg.pose.covariance[i] = 0.0;
		}
		new_odom_num = 0;
	}

	// compute covariance of ndt scan matching result when Kalman filter-based was not done
	if (compute_cov)
	{
		Eigen::MatrixXd R_tmp(6, 6), R(6, 6);
		for (int j = 0; j < 6; j++)
		{
			for (int i = 0; i < 6; i++)  R_tmp(i, j) = hessian[i][j];
		}
		int points_num;
		if (!use_voxel_grid_filter)
			points_num = scan_points_num;
		else
			points_num = filtered_scan_points_num;
		R = R_tmp.inverse();
		R *= ndt_noise_level;
		R *= (2.0 * ndt_error) / (double)(points_num - 3);
		for (int j = 0; j < 6; j++)
		{
			for (int i = 0; i < 6; i++)  ndt_noise[j][i] = R(j, i);
		}
	}

	// publish ndt_frame as PoseWithCovarianceStamped
	geometry_msgs::PoseWithCovarianceStamped ndt_pose_with_cov;
	ndt_pose_with_cov.header.frame_id = "/world";
	ndt_pose_with_cov.header.stamp = header.stamp;
	ndt_pose_with_cov.pose.pose.position.x = ndt_base_point(0);
	ndt_pose_with_cov.pose.pose.position.y = ndt_base_point(1);
	ndt_pose_with_cov.pose.pose.position.z = ndt_base_point(2);
	geometry_msgs::Quaternion ndt_q;
	quaternionTFToMsg(q, ndt_q);
	ndt_pose_with_cov.pose.pose.orientation = ndt_q;
	for (int j = 0; j < 6; j++)
	{
		for (int i = 0; i < 6; i++)	ndt_pose_with_cov.pose.covariance[j * 6 + i] = ndt_noise[j][i];	
	}
	ndt_current_pose_with_cov_pub.publish(ndt_pose_with_cov);

	// draw error elipse of ndt scan matching result
	Eigen::Matrix2d A;
	A << ndt_noise[0][0], ndt_noise[0][1], ndt_noise[1][0], ndt_noise[1][1];
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(A);
	if (es.info() == 0) // eigen values could not be comupted
	{
		double ev1, ev2, chi2 = 9.21034;
		ev1 = es.eigenvalues()(0);
		ev2 = es.eigenvalues()(1);
		if (ev1 >= 0.0 && ev2 >= 0.0) // eigen values was imaginary
		{
			double ra, rb, t;
			if (fabs(ev1) >= fabs(ev2))
			{
				ra = sqrt(chi2 * ev1);
				rb = sqrt(chi2 * ev2);
				t = atan2(es.eigenvectors()(0, 1), es.eigenvectors()(0, 0));
			}
			else
			{
				ra = sqrt(chi2 * ev2);
				rb = sqrt(chi2 * ev1);
				t = atan2(es.eigenvectors()(1, 1), es.eigenvectors()(1, 0));
			}
			visualization_msgs::Marker ee;
			ee.header.stamp = header.stamp;
			ee.header.frame_id = "/world";
			ee.ns = "ndt_error_ellipse_ndt";
			ee.id = 0;
			ee.type = visualization_msgs::Marker::CYLINDER;
			ee.pose.position.x = ndt_base_point(0);
			ee.pose.position.y = ndt_base_point(1);
			ee.pose.position.z = ndt_base_point(2);
			ee.pose.orientation = tf::createQuaternionMsgFromYaw(t);
			ee.scale.x = ra;
			ee.scale.y = rb;
			ee.scale.z = 0.01;
			ee.color.r = 1.0;
			ee.color.g = 0.0;
			ee.color.b = 1.0;
			ee.color.a = 1.0;
			ndt_error_elipse_pub.publish(ee);
		}
	}

#ifdef RECORD_TRAJECTORY
	static FILE *fp;
	if (!fp) fp = fopen("/tmp/ndt_path.txt", "w");
	fprintf(fp, "%lf %lf %lf\n", ndt_pose.x, ndt_pose.y, ndt_pose.z);
#endif

	// compute current velodyne pose
	Eigen::MatrixXd base_2_vel(4, 4);
	ca = cos(vehicle_pose.roll);
	sa = sin(vehicle_pose.roll);
	cb = cos(vehicle_pose.pitch);
	sb = sin(vehicle_pose.pitch);
	cg = cos(vehicle_pose.yaw);
	sg = sin(vehicle_pose.yaw);
	base_2_vel(0, 0) = ca * cb * cg - sa * sg;
	base_2_vel(0, 1) = -ca * cb * sg - sa * cg;
	base_2_vel(0, 2) = ca * sb;
	base_2_vel(0, 3) = vehicle_pose.x;
	base_2_vel(1, 0) = sa * cb * cg + ca * sg;
	base_2_vel(1, 1) = -sa * cb * sg + ca * cg;
	base_2_vel(1, 2) = sa * sb;
	base_2_vel(1, 3) = vehicle_pose.y;
	base_2_vel(2, 0) = -sb * cg;
	base_2_vel(2, 1) = sb * sg;
	base_2_vel(2, 2) = cb;
	base_2_vel(2, 3) = vehicle_pose.z;
	base_2_vel(3, 0) = 0.0;
	base_2_vel(3, 1) = 0.0;
	base_2_vel(3, 2) = 0.0;
	base_2_vel(3, 3) = 1.0;
	Eigen::VectorXd current_vel_point(4);
	current_vel_point = base_2_vel * velodyne_point_in_base_link_frame;

	// current velodyne pose
	pose.x = current_vel_point(0);
	pose.y = current_vel_point(1);
	pose.z = current_vel_point(2);
	pose.theta = vehicle_pose.roll;
	pose.theta2 = vehicle_pose.pitch;
	pose.theta3 = vehicle_pose.yaw;

	// publish current velodyne pose for Autoware (vscan?)
	geometry_msgs::PoseStamped velodyne_pose;
	velodyne_pose.header = header;
	velodyne_pose.pose.position.x = pose.x;
	velodyne_pose.pose.position.y = pose.y;
	velodyne_pose.pose.position.z = pose.z;
	tf::Quaternion velodyne_q;
	velodyne_q.setRPY(pose.theta, pose.theta2, pose.theta3);
	geometry_msgs::Quaternion velodyne_q_msg;
	tf::quaternionTFToMsg(velodyne_q, velodyne_q_msg);
	velodyne_pose.pose.orientation = velodyne_q_msg;
	velodyne_pose_pub.publish(velodyne_pose);

	if (add_map_points)
	{
		scan_transrate(scan_points, map_points, &pose, scan_points_num);
		for (int i = 0; i < scan_points_num; i++)
			map_points_i[i] = scan_points_i[i];
		distance = (key_pose.x - pose.x) * (key_pose.x - pose.x) +
			(key_pose.y - pose.y) * (key_pose.y - pose.y) +
			(key_pose.z - pose.z) * (key_pose.z - pose.z);
		if (g_map_update && (!is_map_exist || (distance > 1 * 1 && scan_points_num > 100)))
		{
			for (int i = 0; i < scan_points_num; i++)
				add_point_map(NDmap,&map_points[i]);
			key_pose = pose;
			is_map_exist = 1;
			if (debug_flag)	fprintf(stderr, "add points to map\n");
		}
	}

#ifdef SAVE_VELODYNE_PCD
	// save velodyne readings in world frame as pcd
	static unsigned int count_pcd;
	static Posture prev_pose_pcd;
	static bool is_first_pcd = true;
	bool save_pcd = false;
	if (is_first_pcd)
	{
		if (count >= 10)
		{
			is_first_pcd = false;
			save_pcd = true;
		}
	}
	else
	{
		distance = (prev_pose_pcd.x - pose.x) * (prev_pose_pcd.x - pose.x) +
			(prev_pose_pcd.y - pose.y) * (prev_pose_pcd.y - pose.y) +
			(prev_pose_pcd.z - pose.z) * (prev_pose_pcd.z - pose.z);
		if (distance > 1 * 1 && scan_points_num > 100)
			save_pcd = true;
	}
	if (save_pcd)
	{
		scan_transrate(scan_points, map_points, &pose, scan_points_num);
		pcl::PointXYZRGB p;
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		for (int i = 0; i < scan_points_num; i++)
		{
			distance = (map_points[i].x - pose.x) * (map_points[i].x - pose.x) +
				(map_points[i].y - pose.y) * (map_points[i].y - pose.y) +
				(map_points[i].z - pose.z) * (map_points[i].z - pose.z);
			if (distance > 80 * 80)
			{
				p.x = map_points[i].x;
				p.y = map_points[i].y;
				p.z = map_points[i].z;
				p.r = p.g = p.b = 255;
				cloud.points.push_back(p);
			}
		}
		if (cloud.points.size() > 0)
		{
			cloud.width = cloud.points.size();
			cloud.height = 1;
			char buf[1024];
			sprintf(buf, "/tmp/PCD/Sense/points_%d.pcd", count_pcd);
			pcl::io::savePCDFileBinary(buf, cloud);
			prev_pose_pcd = pose;
			count_pcd++;
		}
	}
#endif

	// record previous velodyne poses
	prev_pose2 = prev_pose;
	prev_pose = pose;
	if (is_first_time)
	{
		prev_pose2 = prev_pose;
		is_first_time = 0;
	}

	if (debug_flag)
	{
		static std_msgs::Time old_time;
		std_msgs::Time now_time;
		now_time.data = ros::Time::now();
		double delta_time = now_time.data.toSec() - old_time.data.toSec();
		double exec_time = now_time.data.toSec() - start_time.data.toSec();
		int points_num;
		if (!use_voxel_grid_filter)
			points_num = scan_points_num;
		else
			points_num = filtered_scan_points_num;
		fprintf(stderr, "-----estimated result-----\n");
		fprintf(stderr, "x = %.2lf, y = %.2lf, z = %.2lf [m]\n", vehicle_pose.x, vehicle_pose.y, vehicle_pose.z);
		fprintf(stderr, "roll = %.2lf, pitch = %.2lf, yaw = %.2lf [deg]\n", vehicle_pose.roll * 180.0 / M_PI, vehicle_pose.pitch * 180.0 / M_PI, vehicle_pose.yaw * 180.0 / M_PI);
		fprintf(stderr, "delta time = %.4lf, execution time = %.4lf [sec]\n", delta_time, exec_time);
		fprintf(stderr, "ndt error = %lf, points num = %d\n", ndt_error, points_num);
		fprintf(stderr, "\n");
		old_time = now_time;
	}
	count++;
}

void dummy_odom_init(void)
{
	stamped_pose_t p;
	p.is_data = false;
	stamped_vehicle_poses.resize(ODOM_BUF_SIZE, p);
	for (int i = 0; i < 36; i++)
	{
		if (is_slam_mode)
			vehicle_pose_msg.pose.covariance[i] = 0.0;
		else
			reset_odom_covariance();
	}
	is_odom_initialized = true;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &input)
{
	static pose_t odom_pose;

	if (!is_odom_initialized)
	{
		// Record previous state
		previous_odom = *input;
		stamped_pose_t p;
		p.is_data = false;
		stamped_vehicle_poses.resize(ODOM_BUF_SIZE, p);
		if (is_slam_mode)
		{
			for (int i = 0; i < 36; i++)
				vehicle_pose_msg.pose.covariance[i] = 0.0;
		}
		else
		{
			reset_odom_covariance();
		}
		is_odom_initialized = true;
		odom_pose = vehicle_pose;
	}
	else
	{
		static int count;
		count++;
		if (count < odom_interval)
			return;
		count = 0;
		// Update odom_pose
		pose_t old_pose = vehicle_pose;
		tf::Quaternion prev_odom_q(previous_odom.pose.pose.orientation.x,
			previous_odom.pose.pose.orientation.y,
			previous_odom.pose.pose.orientation.z,
			previous_odom.pose.pose.orientation.w);
		double prev_odom_roll, prev_odom_pitch, prev_odom_yaw;
		tf::Matrix3x3(prev_odom_q).getRPY(prev_odom_roll, prev_odom_pitch, prev_odom_yaw);
		tf::Quaternion curr_odom_q(input->pose.pose.orientation.x,
			input->pose.pose.orientation.y,
			input->pose.pose.orientation.z,
			input->pose.pose.orientation.w);
		double curr_odom_roll, curr_odom_pitch, curr_odom_yaw;
		tf::Matrix3x3(curr_odom_q).getRPY(curr_odom_roll, curr_odom_pitch, curr_odom_yaw);
		double dx = input->pose.pose.position.x - previous_odom.pose.pose.position.x;
		double dy = input->pose.pose.position.y - previous_odom.pose.pose.position.y;
		double delta_dist = sqrt(dx * dx + dy * dy);
		double delta_yaw = curr_odom_yaw - prev_odom_yaw;
		if (delta_yaw < -M_PI)  delta_yaw += 2.0 * M_PI;
		if (delta_yaw > M_PI)   delta_yaw -= 2.0 * M_PI;
		if (fabs(delta_dist) < 0.001)	delta_yaw = 0.0;	// for gyro odometry
		double delta_x = dx * cos(-prev_odom_yaw) - dy * sin(-prev_odom_yaw);
		double delta_y = dx * sin(-prev_odom_yaw) + dy * cos(-prev_odom_yaw);
		if (delta_x < 0.0)	delta_dist *= -1.0;
		double tmp_x = delta_x * cos(vehicle_pose.yaw) - delta_y * sin(vehicle_pose.yaw) + vehicle_pose.x;
		double tmp_y = delta_x * sin(vehicle_pose.yaw) + delta_y * cos(vehicle_pose.yaw) + vehicle_pose.y;
		double tmp_yaw = vehicle_pose.yaw + delta_yaw;
		if (tmp_yaw < -M_PI)  tmp_yaw += 2.0 * M_PI;
		if (tmp_yaw > M_PI)   tmp_yaw -= 2.0 * M_PI;
		vehicle_pose.x = tmp_x;
		vehicle_pose.y = tmp_y;
		vehicle_pose.yaw = tmp_yaw;
		odom_pose.x += delta_dist * cos(odom_pose.yaw);
		odom_pose.y += delta_dist * sin(odom_pose.yaw);
		odom_pose.yaw += delta_yaw;
		if (odom_pose.yaw < -M_PI)
			odom_pose.yaw += 2.0 * M_PI;
		if (odom_pose.yaw > M_PI)
			odom_pose.yaw -= 2.0 * M_PI;

#ifdef RECORD_TRAJECTORY
		static FILE *fp;
		if (!fp)  fp = fopen("/tmp/filtered_path.txt", "w");
		fprintf(fp, "%lf %lf %lf\n", vehicle_pose.x, vehicle_pose.y, vehicle_pose.z);
//		fprintf(fp, "%lf %lf %lf\n", odom_pose.x, odom_pose.y, odom_pose.z);
#endif

		// Update error covariance
		double dd = delta_dist * delta_dist, yy = delta_yaw * delta_yaw;
		Eigen::MatrixXd V(3, 2), N(2, 2), Q(3, 3), G(3, 3), SO(3, 3), SN(3, 3);
		V(0, 0) = cos(old_pose.yaw);
		V(0, 1) = 0.0;
		V(1, 0) = sin(old_pose.yaw);
		V(1, 1) = 0.0;
		V(2, 0) = 0.0;
		V(2, 1) = 1.0;
		N(0, 0) = odom_noise1 * dd + odom_noise2 * yy;
		N(0, 1) = 0.0;
		N(1, 0) = 0.0;
		N(1, 1) = odom_noise3 * dd + odom_noise4 * yy;
		Q = V * N * V.transpose();

		// new noise (it is not appeared in general kalman filter algorithm)
		Q(0, 0) += (odom_noise5 * dd + odom_noise6 * yy) * sin(old_pose.yaw) * sin(old_pose.yaw);
		Q(0, 1) += (odom_noise5 * dd + odom_noise6 * yy) * cos(old_pose.yaw) * sin(old_pose.yaw);
		Q(1, 0) += (odom_noise5 * dd + odom_noise6 * yy) * cos(old_pose.yaw) * sin(old_pose.yaw);
		Q(1, 1) += (odom_noise5 * dd + odom_noise6 * yy) * cos(old_pose.yaw) * cos(old_pose.yaw);

		G(0, 0) = 1.0;
		G(0, 1) = 0.0;
//		G(0, 2) = -delta_dist * sin(old_pose.yaw);
		G(0, 2) = delta_dist * cos(old_pose.yaw);
		G(1, 0) = 0.0;
		G(1, 1) = 1.0;
//		G(1, 2) = delta_dist * cos(old_pose.yaw);
		G(1, 2) = delta_dist * sin(old_pose.yaw);
		G(2, 0) = 0.0;
		G(2, 1) = 0.0;
		G(2, 2) = 1.0;
		SO(0, 0) = vehicle_pose_msg.pose.covariance[0 * 6 + 0];
		SO(0, 1) = vehicle_pose_msg.pose.covariance[0 * 6 + 1];
		SO(0, 2) = vehicle_pose_msg.pose.covariance[0 * 6 + 5];
		SO(1, 0) = vehicle_pose_msg.pose.covariance[1 * 6 + 0];
		SO(1, 1) = vehicle_pose_msg.pose.covariance[1 * 6 + 1];
		SO(1, 2) = vehicle_pose_msg.pose.covariance[1 * 6 + 5];
		SO(2, 0) = vehicle_pose_msg.pose.covariance[5 * 6 + 0];
		SO(2, 1) = vehicle_pose_msg.pose.covariance[5 * 6 + 1];
		SO(2, 2) = vehicle_pose_msg.pose.covariance[5 * 6 + 5];
		SN = G * SO * G.transpose() + Q;

		// Copy odom_pose to vehicle_pose_msg
		geometry_msgs::Quaternion vehicle_quat = tf::createQuaternionMsgFromRollPitchYaw(vehicle_pose.roll, vehicle_pose.pitch, vehicle_pose.yaw); 	
		vehicle_pose_msg.header = input->header;
		vehicle_pose_msg.pose.pose.position.x = vehicle_pose.x;
		vehicle_pose_msg.pose.pose.position.y = vehicle_pose.y;
		vehicle_pose_msg.pose.pose.position.z = vehicle_pose.z;
		vehicle_pose_msg.pose.pose.orientation = vehicle_quat;
		vehicle_pose_msg.pose.covariance[0 * 6 + 0] = SN(0, 0);
		vehicle_pose_msg.pose.covariance[0 * 6 + 1] = SN(0, 1);
		vehicle_pose_msg.pose.covariance[0 * 6 + 5] = SN(0, 2);
		vehicle_pose_msg.pose.covariance[1 * 6 + 0] = SN(1, 0);
		vehicle_pose_msg.pose.covariance[1 * 6 + 1] = SN(1, 1);
		vehicle_pose_msg.pose.covariance[1 * 6 + 5] = SN(1, 2);
		vehicle_pose_msg.pose.covariance[5 * 6 + 0] = SN(2, 0);
		vehicle_pose_msg.pose.covariance[5 * 6 + 1] = SN(2, 1);
		vehicle_pose_msg.pose.covariance[5 * 6 + 5] = SN(2, 2);

		// publish current pose as geometry_msgs::PoseStamped
		geometry_msgs::PoseStamped filtered_current_pose;
		filtered_current_pose.header = input->header;
		filtered_current_pose.pose.position = vehicle_pose_msg.pose.pose.position;
		filtered_current_pose.pose.orientation = vehicle_quat;
		filtered_current_pose_pub.publish(filtered_current_pose);

		// Record odometry data for fusion
		stamped_pose_t st_pose;
		st_pose.p = vehicle_pose;
		st_pose.time = input->header.stamp.toSec();
		st_pose.is_data = true;
		st_pose.varxx = SN(0, 0);
		st_pose.varyy = SN(1, 1);
		st_pose.vartt = SN(2, 2);
		st_pose.varxy = SN(0, 1);
		st_pose.varyt = SN(1, 2);
		st_pose.vartx = SN(2, 0);
		stamped_vehicle_poses.insert(stamped_vehicle_poses.begin(), st_pose);
		stamped_vehicle_poses.resize(ODOM_BUF_SIZE);
		new_odom_num++;

		// send transform interpolated pose between NDT localization frames
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		tf::Quaternion odom_q(vehicle_pose_msg.pose.pose.orientation.x,
			vehicle_pose_msg.pose.pose.orientation.y,
			vehicle_pose_msg.pose.pose.orientation.z,
			vehicle_pose_msg.pose.pose.orientation.w);
		transform.setOrigin(tf::Vector3(vehicle_pose.x, vehicle_pose.y, vehicle_pose.z));
		transform.setRotation(odom_q);
		br.sendTransform(tf::StampedTransform(transform, input->header.stamp, "/world", filtered_ndt_frame_id));

		// Record previous state
		previous_odom = *input;

		// Publish current vehicle pose message including pose and error covariance
		vehicle_pose_pub.publish(vehicle_pose_msg);

		// Publish error ellipse marker (optional)
		double a, b, c, d;
		a = vehicle_pose_msg.pose.covariance[0 * 6 + 0];
		b = vehicle_pose_msg.pose.covariance[0 * 6 + 1];
		c = vehicle_pose_msg.pose.covariance[1 * 6 + 0];
		d = vehicle_pose_msg.pose.covariance[1 * 6 + 1];
		Eigen::Matrix2d A;
		A << a, b, c, d;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(A);
		if (es.info() != 0) // eigen values could not be comupted
			return;
		double ev1, ev2, chi2 = 9.21034;
		ev1 = es.eigenvalues()(0);
		ev2 = es.eigenvalues()(1);
		if (ev1 < 0.0 || ev2 < 0.0) // eigen values was imaginary
			return;
		double ra, rb, t;
		if (fabs(ev1) >= fabs(ev2))
		{
			ra = sqrt(chi2 * ev1);
			rb = sqrt(chi2 * ev2);
			t = atan2(es.eigenvectors()(0, 1), es.eigenvectors()(0, 0));
		}
		else
		{
			ra = sqrt(chi2 * ev2);
			rb = sqrt(chi2 * ev1);
			t = atan2(es.eigenvectors()(1, 1), es.eigenvectors()(1, 0));
		}
		visualization_msgs::Marker ee;
		ee.header.stamp = input->header.stamp;
		ee.header.frame_id = "/world";
		ee.ns = "vehicle_error_ellipse_ndt";
		ee.id = 0;
		ee.type = visualization_msgs::Marker::CYLINDER;
		ee.pose.position = vehicle_pose_msg.pose.pose.position;
		ee.pose.orientation = tf::createQuaternionMsgFromYaw(t);
		ee.scale.x = ra;
		ee.scale.y = rb;
		ee.scale.z = 0.01;
		ee.color.a = 1.0;
		ee.color.r = 1.0;
		ee.color.g = 1.0;
		ee.color.b = 0.0;
		error_elipse_pub.publish(ee);
	}
}

void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	tf::StampedTransform transform;

	try
	{
		ros::Time now = ros::Time(0);
		tf_listener->waitForTransform("/world", "/world", now, ros::Duration(10.0));
		tf_listener->lookupTransform("/world", "/world", now, transform);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s", ex.what());
	}
	tf::Quaternion q(msg->pose.pose.orientation.x, 
		msg->pose.pose.orientation.y, 
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	vehicle_pose.x = msg->pose.pose.position.x + transform.getOrigin().x();
	vehicle_pose.y = msg->pose.pose.position.y + transform.getOrigin().y();
	vehicle_pose.z = msg->pose.pose.position.z + transform.getOrigin().z();
	// how to estimate z pose efficiently?
	vehicle_pose.z = msg->pose.pose.position.z + transform.getOrigin().z();
	m.getRPY(vehicle_pose.roll, vehicle_pose.pitch, vehicle_pose.yaw);
	reset_odom_covariance();
	prev_pose.x = vehicle_pose.x;
	prev_pose.y = vehicle_pose.y;
	prev_pose.z = vehicle_pose.z;
	prev_pose.theta  = vehicle_pose.roll;
	prev_pose.theta2 = vehicle_pose.pitch;
	prev_pose.theta3 = vehicle_pose.yaw;
	prev_pose2 = prev_pose;
	is_first_time = true;
	if (debug_flag)
	{
		fprintf(stderr, "-----pose resetting by 2d pose estimate-----\n");
		fprintf(stderr, "x = %.2lf, y = %.2lf, z = %.2lf [m]\n", vehicle_pose.x, vehicle_pose.y, vehicle_pose.z);
		fprintf(stderr, "roll = %.2lf, pitch = %.2lf, yaw = %.2lf [deg]\n", vehicle_pose.roll * 180.0 / M_PI, vehicle_pose.pitch * 180.0 / M_PI, vehicle_pose.yaw * 180.0 / M_PI);
		fprintf(stderr, "\n");
	}
}

/*add point to ndcell */
int add_point_covariance(NDPtr nd,PointPtr p){

  /*add data num*/
  nd->num ++;
  nd->flag = 0; /*need to update*/
  //printf("%d \n",nd->num);
  
  /*calcurate means*/
  nd->m_x += p->x;
  nd->m_y += p->y;
  nd->m_z += p->z;

  /*calcurate covariances*/
  nd->c_xx += p->x*p->x;
  nd->c_yy += p->y*p->y;
  nd->c_zz += p->z*p->z;

  nd->c_xy += p->x*p->y;
  nd->c_yz += p->y*p->z;
  nd->c_zx += p->z*p->x;	

  return 1;
}


/*calcurate covariances*/
int update_covariance(NDPtr nd){
  double a,b,c;/*for calcurate*/
  if(!nd->flag){/*need calcurate?*/    
		/*means*/
    nd->mean.x =a= nd->m_x / nd->num; 
    nd->mean.y =b= nd->m_y / nd->num; 
    nd->mean.z =c= nd->m_z / nd->num; 
    
    /*covariances*/
    nd->covariance[0][0] = (nd->c_xx -2*a*nd->m_x )/ nd->num + a*a;
    nd->covariance[1][1] = (nd->c_yy -2*b*nd->m_y )/ nd->num + b*b;
    nd->covariance[2][2] = (nd->c_zz -2*c*nd->m_z )/ nd->num + c*c;
    nd->covariance[0][1]=nd->covariance[1][0] = (nd->c_xy - nd->m_x*b - nd->m_y*a)/ nd->num+ a*b;
    nd->covariance[1][2]=nd->covariance[2][1] = (nd->c_yz - nd->m_y*c - nd->m_z*b)/ nd->num+ b*c;
    nd->covariance[2][0]=nd->covariance[0][2] = (nd->c_zx - nd->m_z*a - nd->m_x*c)/ nd->num+ c*a;
    nd->sign=0;	
    nd->flag = 1;  /*this ND updated*/
    if(nd->num >= 5){    
      if(round_covariance(nd)==1){
	if(ginverse_matrix3d(nd->covariance, nd->inv_covariance))
	  nd->sign=1;
      }
    }
  }

  return 1;
}


/*add point to ndmap*/
int add_point_map(NDMapPtr ndmap, PointPtr point){
  int x,y,z,i;
  NDPtr  *ndp[8];  

  /*
    
  +---+---+
  |   |   |
  +---+---+
  |   |###|
  +---+---+
  
  */
  
  /*mapping*/
  x = (point->x / ndmap->size) + ndmap->x/2;
  y = (point->y / ndmap->size) + ndmap->y/2;
  z = (point->z / ndmap->size) + ndmap->z/2;
    
  /*clipping*/
  if(x < 1 || x >=  ndmap->x )return 0;
  if(y < 1 || y >=  ndmap->y )return 0;
  if(z < 1 || z >=  ndmap->z )return 0;
  
  /*select root ND*/
  ndp[0] = ndmap->nd + x*ndmap->to_x + y*ndmap->to_y +z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;
 
  /*add  point to map */
  for(i = 0;i < 8;i++){
    if((*ndp[i])==0)*ndp[i]=add_ND();
    if((*ndp[i])!=0)add_point_covariance(*ndp[i], point);
  }
  
  if(ndmap->next){
    add_point_map(ndmap->next , point);  
  }
    
  return 0;		
}

/*get nd cell at point*/
int get_ND(NDMapPtr ndmap,PointPtr point, NDPtr *nd,int  ndmode ){
  int x,y,z;
  int i;
  NDPtr *ndp[8];

  /*
    
  +---+---+
  |   |   |
  +---+---+
  |   |###|
  +---+---+
  
  *//*
  layer = layer_select;
  while(layer > 0){
    if(ndmap->next)ndmap = ndmap->next;
    layer--;
  }
    */
  /*mapping*/
  if(ndmode <3 ){
    x = (double)((point->x / ndmap->size) + ndmap->x/2 -0.5);
    y = (double)((point->y / ndmap->size) + ndmap->y/2 -0.5);
    z = (double)((point->z / ndmap->size) + ndmap->z/2 -0.5);  
  }else{
    x = (point->x / ndmap->size) + ndmap->x/2 ;
    y = (point->y / ndmap->size) + ndmap->y/2 ;
    z = (point->z / ndmap->size) + ndmap->z/2 ; 
  }
  
  /*clipping*/
  if(x < 1 || x >=  ndmap->x )return 0;
  if(y < 1 || y >=  ndmap->y )return 0;
  if(z < 1 || z >=  ndmap->z )return 0;
  

  /*select root ND*/
  ndp[0] = ndmap->nd + x*ndmap->to_x + y*ndmap->to_y +z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1; 
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;
  
  for(i = 0;i < 8;i ++){
    if(*ndp[i] != 0){
      if(!(*ndp[i])->flag)update_covariance(*ndp[i]);
      nd[i]=*ndp[i];
    }else {
      nd[i]=NDs;
      //return 0;
    }
  }

   
  return 1;		
}


NDPtr add_ND(void){
  NDPtr ndp;
  //int m;

  if(NDs_num>=MAX_ND_NUM){
    printf("over flow\n");
    return 0;
  }
  
  ndp= NDs+NDs_num;
  NDs_num++;
  
  ndp->flag = 0;
  ndp->sign = 0;
  ndp->num  = 0;
  ndp->m_x  = 0;
  ndp->m_y  = 0;
  ndp->m_z  = 0;
  ndp->c_xx = 0;
  ndp->c_yy = 0;
  ndp->c_zz = 0;
  ndp->c_xy = 0;
  ndp->c_yz = 0;
  ndp->c_zx = 0;
  ndp->w = 1;
  ndp->is_source = 0;
  
  return ndp;
}

NDMapPtr initialize_NDmap_layer(int layer, NDMapPtr child){
  int i,j,k,i2,i3,m;
  int x,y,z;
  NDPtr    *nd,*ndp;
  NDMapPtr ndmap;
  
  i2 = i3 = 0;
  printf("Initializing...layer %d\n",layer);
  
  
  x = (g_map_x >> layer)+1;
  y = (g_map_y >> layer)+1; 
  z = (g_map_z >> layer)+1;
  
  nd = (NDPtr*)malloc(x*y*z*sizeof(NDPtr));
  ndmap= (NDMapPtr)malloc(sizeof(NDMap));
  
  ndmap->x = x;
  ndmap->y = y;
  ndmap->z = z;
  ndmap->to_x = y*z;
  ndmap->to_y = z;
  ndmap->layer = layer;
  ndmap->nd = nd;
  ndmap->next = child;
  ndmap->size = g_map_cellsize * ((int)1<<layer);
  printf("size %f\n",ndmap->size);
  
  ndp = nd;

  for(i = 0; i < x; i++){
    for(j = 0; j < y; j++){
      for(k = 0; k < z; k++){
	*ndp=0;
	ndp++;
      }
    }
  }

  return ndmap;
}

NDMapPtr initialize_NDmap(void){
  int i;
  NDMapPtr ndmap;
  NDPtr null_nd;

  printf("Initializing");
  ndmap = 0;

  //init NDs
  NDs=(NDPtr)malloc(sizeof(NormalDistribution)*MAX_ND_NUM);
  NDs_num=0;
  
  null_nd= add_ND();

  for(i = LAYER_NUM-1;i >= 0;i--){
    ndmap = initialize_NDmap_layer(i,ndmap);

    /*progress dots*/
    printf("layer %d\n",i);
  }


  printf("done\n");

  return ndmap;
}


int round_covariance(NDPtr nd){
  double v[3][3],a;

  eigenvecter_matrix3d(nd->covariance,v,nd->l);
  //  print_matrix3d(v);
  if(fabs(v[0][0]*v[0][0]+v[1][0]*v[1][0]+v[2][0]*v[2][0]-1) >0.1)printf("!1");
  if(fabs(v[0][0]*v[0][1]+v[1][0]*v[1][1]+v[2][0]*v[2][1]) >0.01)printf("!01");
  if(fabs(v[0][1]*v[0][2]+v[1][1]*v[1][2]+v[2][1]*v[2][2]) >0.01)printf("!02");
  if(fabs(v[0][2]*v[0][0]+v[1][2]*v[1][0]+v[2][2]*v[2][0]) >0.01)printf("!03");

  a = fabs(nd->l[1]/nd->l[0]);
  if(a <0.001){
    return 0;
    if(nd->l[1] > 0)
      nd->l[1] = fabs(nd->l[0])/10.0;
    else 
      nd->l[1] = -fabs(nd->l[0])/10.0;

  
    a = fabs(nd->l[2]/nd->l[0]);
    if(a <0.01){
      if(nd->l[2] > 0)
	nd->l[2] = fabs(nd->l[0])/10.0;
      else 
	nd->l[2] =-fabs(nd->l[0])/10.0;
     
     
    }
    matrix3d_eigen(v,nd->l[0],nd->l[1],nd->l[2],nd->covariance);
  }
  return 1;
}


double probability_on_ND(NDPtr nd,double xp,double yp,double zp){
  //  double xp,yp,zp;
  double e,val;
  
  if(nd->num < 5)return 0;
  /*
  xp = x - nd->mean.x;
  yp = y - nd->mean.y;
  zp = z - nd->mean.z;
  */
  val = (xp*xp*nd->inv_covariance[0][0] +
	   yp*yp*nd->inv_covariance[1][1] + 
	   zp*zp*nd->inv_covariance[2][2] +
	   2.0* xp*yp*nd->inv_covariance[0][1] +
	   2.0* yp*zp*nd->inv_covariance[1][2] +
	   2.0* zp*xp*nd->inv_covariance[2][0] )
	  /2.0;
  ndt_error += val;
  e = exp(-val);

  if(e > 1)return 1;
  if(e < 0)return 0;  
  return(e);
}

void load(char *name){
  FILE *fp;
  double x,y,z,q;
  Point p;

  fp=fopen(name,"r");

  while(fscanf(fp,"%lf %lf %lf %lf",&y,&x,&z,&q)!=EOF){//x,y swaped
    p.x=(x-g_map_center_x)*cos(-g_map_rotation)-(y-g_map_center_y)*sin(-g_map_rotation);
    p.y=(x-g_map_center_x)*sin(-g_map_rotation)+(y-g_map_center_y)*cos(-g_map_rotation);
    p.z=z-g_map_center_z;
    
    add_point_map(NDmap, &p);
  }
  fclose(fp);
}

void save_nd_map(char* name){
  int i,j,k,layer;
  NDData nddat;
  NDMapPtr ndmap;
  NDPtr *ndp;
  FILE *ofp;
  
  //for pcd 
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ p;
  
  ndmap=NDmap;
  ofp=fopen(name,"w");
  
  for(layer=0;layer<2;layer++){
    ndp = ndmap->nd;
    /*�쥤�䡼�ν�����*/
    for(i = 0; i < ndmap->x; i++){
      for(j = 0; j < ndmap->y; j++){
	for(k = 0; k < ndmap->z; k++){
	  if(*ndp){
	    update_covariance(*ndp); 
	    nddat.nd = **ndp;
	    nddat.x =i;
	    nddat.y =j;
	    nddat.z =k;
	    nddat.layer= layer;
	    
	    fwrite(&nddat,sizeof(NDData),1,ofp);

	    //regist the point to pcd data;
	    p.x = (*ndp)->mean.x;
	    p.y = (*ndp)->mean.y;
	    p.z = (*ndp)->mean.z;
	    cloud.points.push_back(p);
    
	  }
	  ndp++;
	}
      }
    }
    ndmap=ndmap->next;
    
  }
  printf("done\n");
  fclose(ofp);

	//save pcd
	cloud.width = cloud.points.size();
	cloud.height = 1;
	pcl::io::savePCDFileASCII("/tmp/means.pcd", cloud);
	fprintf(stderr, "%d ND boxels were recorded\n", (int)cloud.points.size());  
}

//load ndt setting file
int load_ndt_ini(char* name){
  FILE *ifp;
  char ndmap_name[200];
  char ndmap_dir[200];
  int i;

  ifp=fopen(name,"r");
  if(!ifp)return 0;

  //map path
  if(fscanf(ifp,"%s",ndmap_name)==EOF)return 0;
  
  for(i=strlen(name)-1;i>0;i--){
    if(name[i]=='/'){
      ndmap_dir[i+1]=0;
      break;
    }
  }
  for(;i>=0;i--)ndmap_dir[i]=name[i];
  sprintf(g_ndmap_name,"%s%s",ndmap_dir,ndmap_name);
  printf("%s\n",g_ndmap_name);
  //map size
  if(fscanf(ifp,"%d %d %d %lf",
	    &g_map_x,&g_map_y,&g_map_z,&g_map_cellsize)==EOF)return 0;
  //map center
  if(fscanf(ifp,"%lf %lf %lf %lf",
	    &g_map_center_x,&g_map_center_y,&g_map_center_z,
	    &g_map_rotation)==EOF)return 0;
  //use gnss
  if(fscanf(ifp,"%d", &g_use_gnss)==EOF)return 0;
  if(!g_use_gnss){
    if(fscanf(ifp,"%lf %lf %lf %lf %lf %lf",
	      &g_ini_x,&g_ini_y,&g_ini_z,
	      &g_ini_roll,&g_ini_pitch,&g_ini_yaw)==EOF)return 0;
  }

  //
  return 1;
}

int load_nd_map(char* name){
  int i,j,k,layer;
  NDData nddat;
  NDMapPtr ndmap[2];
  NDPtr ndp;
  FILE *ifp;
	int c=0;
  //  FILE *logfp;


  ndmap[0]=NDmap;
  ndmap[1]=NDmap->next;

  ifp=fopen(name,"r");
  if(!ifp)return 0;

  while(fread(&nddat,sizeof(NDData),1,ifp)>0){  
    ndp=add_ND();
    *ndp = nddat.nd;
    *(ndmap[nddat.layer]->nd + nddat.x*ndmap[nddat.layer]->to_x + nddat.y*ndmap[nddat.layer]->to_y + nddat.z)
      = ndp;
    ndp->flag=0;
    update_covariance(ndp); 
    //fprintf(logfp,"%f %f %f \n",ndp->mean.x, ndp->mean.y, ndp->mean.z);
    c++;
    if(c==100000){
      fprintf(stderr,".");
      c=0;
    }
  }

	fprintf(stderr, "\n%d ND voxels were loaded\n",NDs_num);
	fclose(ifp);
	return 1;
}

void *nd_map_mean_points_publisher(void *obj)
{
	int i, j, k, layer;
	NDData nddat;
	NDMapPtr ndmap;
	NDPtr *ndp;
	geometry_msgs::Point32 p;
	sensor_msgs::PointCloud map;
	ros::Rate loop_rate(0.1);

	while (ros::ok())
	{
		ndmap = NDmap;
		for (layer = 0; layer < 2; layer++)
		{
			ndp = ndmap->nd;
			for (i = 0; i < ndmap->x; i++)
			{
				for (j = 0; j < ndmap->y; j++)
				{
					for (k = 0; k < ndmap->z; k++)
					{
						if (*ndp)
						{
							update_covariance(*ndp); 
							nddat.nd = **ndp;
							nddat.x = i;
							nddat.y = j;
							nddat.z = k;
							nddat.layer = layer;
							p.x = (*ndp)->mean.x;
							p.y = (*ndp)->mean.y;
							p.z = (*ndp)->mean.z;
							map.points.push_back(p);
						}
						ndp++;
					}
				}
			}
			ndmap = ndmap->next;
		}

		map.header.stamp = ros::Time::now();
		map.header.frame_id = "/world";
		map_pub.publish(map);
		loop_rate.sleep();
		map.points.clear();
	}
}

void record_nd_map_as_txt(void)
{
	int i, j, k, layer;
	double x, y, z;
	NDData nddat;
	NDMapPtr ndmap;
	NDPtr *ndp;
	FILE *fp;

	ndmap = NDmap;
	fp = fopen("/tmp/ndmap.txt", "w");
	if (fp == NULL)	return;
	for (layer = 0; layer < 2; layer++)
	{
		ndp = ndmap->nd;
		for (i = 0; i < ndmap->x; i++)
		{
			for (j = 0; j < ndmap->y; j++)
			{
				for (k = 0; k < ndmap->z; k++)
				{
					if (*ndp)
					{
						update_covariance(*ndp); 
						nddat.nd = **ndp;
						nddat.x = i;
						nddat.y = j;
						nddat.z = k;
						nddat.layer = layer;
						x = (*ndp)->mean.x;
						y = (*ndp)->mean.y;
						z = (*ndp)->mean.z;
						if (-10.0 < z && z < -5.0)
							fprintf(fp, "%lf %lf %lf\n", x, y, z);
					}
					ndp++;
				}
			}
		}
		ndmap = ndmap->next;
	}
	fclose(fp);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ndt3d_with_odom");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	char nd_filename[500];

  //load setting files
	if (argc > 1)
	{
		if (!load_ndt_ini(argv[1]))
		{
			ROS_ERROR("setting file error\n");
			return 0;
		}
	}
	else
	{
		ROS_ERROR("cannot find setting file\n");
		return 0;
	}

	// print ndt scan mathcing parameters
	fprintf(stderr, "ND map size\nx = %d(%5.2fm) \ny = %d(%5.2fm) \nz = %d(%5.2fm)\n",//memory size = %dMB(Map%dMB)\n",
		g_map_x, g_map_x * g_map_cellsize,
		g_map_y, g_map_y * g_map_cellsize,
		g_map_z, g_map_z * g_map_cellsize);
//		(int)(g_map_x * g_map_y * g_map_z * (1 + 1 / 64.) * sizeof(NDPtr) +
//		MAX_ND_NUM * sizeof(NormalDistribution)) / (1024 * 1024),
//		(int)(g_map_x * g_map_y * g_map_z * (1 + 1 / 64.) * sizeof(NDPtr) +
//		MAX_ND_NUM * sizeof(NormalDistribution)) / (1024 * 1024));
	sprintf(scan_file_base_name, "%s", argv[1]);

	// initialize(clear) NDmap data
	NDmap = initialize_NDmap();

	// set initial poses
	prev_pose.x = (g_ini_x - g_map_center_x) * cos(-g_map_rotation) - (g_ini_y - g_map_center_y) * sin(-g_map_rotation);
	prev_pose.y = (g_ini_x - g_map_center_x) * sin(-g_map_rotation) + (g_ini_y - g_map_center_y) * cos(-g_map_rotation);
	prev_pose.z = g_ini_z - g_map_center_z;
	prev_pose.theta  = g_ini_roll;
	prev_pose.theta2 = g_ini_pitch;
	prev_pose.theta3 = g_ini_yaw - g_map_rotation;
	prev_pose2 = prev_pose;
	is_first_time = 1;
	vehicle_pose.x = prev_pose.x;
	vehicle_pose.y = prev_pose.y;
	vehicle_pose.z = prev_pose.z;
	vehicle_pose.roll = prev_pose.theta;
	vehicle_pose.pitch = prev_pose.theta2;
	vehicle_pose.yaw = prev_pose.theta3;

	// load map
	if (argc > 2)
	{
		fprintf(stderr, "load point cloud map\n");    
		for (int i = 2; i < argc; i++)
		{
			fprintf(stderr, "load(%d/%d) %s\n", i - 2, argc - 2, argv[i]);
			load(argv[i]);  
		}
		save_nd_map(g_ndmap_name);
		is_map_exist = 1;
	}
	else if (argc > 1)
	{
		fprintf(stderr, "start ND map load\n");
		if (load_nd_map(g_ndmap_name))
		is_map_exist = 1;
	}
	base_frame_id="/base_link";
	velodyne_frame_id="/velodyne";
	velodyne_topic="velodyne_points";
	ndt_frame_id="ndt_frame";
	filtered_ndt_frame_id="filtered_ndt_frame";
	// read ros parameters
	np.getParam("use_voxel_grid_filter", use_voxel_grid_filter);
	np.getParam("voxel_leaf_size", voxel_leaf_size);
	np.getParam("odom_topic_name", odom_topic_name);
	np.getParam("use_odometry", use_odometry);
	np.getParam("is_flat_surface", is_flat_surface);
	np.getParam("odom_interval", odom_interval);
	np.getParam("odom_noise1", odom_noise1);
	np.getParam("odom_noise2", odom_noise2);
	np.getParam("odom_noise3", odom_noise3);
	np.getParam("odom_noise4", odom_noise4);
	np.getParam("odom_noise5", odom_noise5);
	np.getParam("odom_noise6", odom_noise6);
	np.getParam("use_fusion", use_fusion);
	np.getParam("ndt_noise_level", ndt_noise_level);
	np.getParam("use_mahalanobis_dist", use_mahalanobis_dist);
	np.getParam("mahalanobis_dist_threshold", mahalanobis_dist_threshold);
	np.getParam("euclid_dist_threshold", euclid_dist_threshold);
	np.getParam("yaw_andle_threshold", yaw_andle_threshold);
	np.getParam("min_trace", min_trace);
	np.getParam("save_map", save_map);
	np.getParam("add_map_points", add_map_points);
	np.getParam("is_slam_mode", is_slam_mode);
	np.getParam("debug_flag", debug_flag);
	np.getParam("base_frame_id", base_frame_id);
	np.getParam("velodyne_frame_id", velodyne_frame_id);
	np.getParam("ndt_frame_id", ndt_frame_id);
	np.getParam("filtered_ndt_frame_id", filtered_ndt_frame_id);
	np.getParam("velodyne_topic",velodyne_topic);

	ndt_noise_level /= g_map_cellsize;
	if (voxel_leaf_size < 0.1)	voxel_leaf_size = 0.1; // minimum size of the filter
	if (is_slam_mode)	add_map_points = true, g_use_gnss = 0;



	ros::Subscriber points_sub = n.subscribe(velodyne_topic, 1000, velodyne_callback);

	ros::Subscriber odom_sub = n.subscribe(odom_topic_name.c_str(), ODOM_BUF_SIZE, odom_callback);
	if (!use_odometry)	dummy_odom_init();
	ros::Subscriber initial_pose_sub = n.subscribe("initialpose", 1000, initial_pose_callback);
	filtered_scan_pub = n.advertise<sensor_msgs::PointCloud2>("vg_filtered_scan_points", 10);
	map_pub = n.advertise<sensor_msgs::PointCloud>("nd_map_mean_points", 1);
	ndt_current_velodyne_pose_pub = n.advertise<geometry_msgs::PoseStamped>("ndt_current_velodyne_pose", 100); // 10 Hz
	ndt_current_pose_with_cov_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("ndt_current_pose_with_covariance", 100); // 10 Hz
	filtered_current_pose_pub = n.advertise<geometry_msgs::PoseStamped>("filtered_ndt_current_pose", 500); // 100 Hz
	vehicle_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("filtered_ndt_current_pose_with_covariance", 500); // 100 Hz
	error_elipse_pub = n.advertise<visualization_msgs::Marker>("vehicle_error_elipse", 100);
	ndt_error_elipse_pub = n.advertise<visualization_msgs::Marker>("ndt_error_elipse", 100);
	velodyne_pose_pub = n.advertise<geometry_msgs::PoseStamped>("localizer_pose", 100); // velodyne pose (for vscan) 10 Hz
	ndt_result_pub = n.advertise<ndt3d::NDTResult>("ndt_localization_result", 100); // 10 Hz (publish at the same timing of localization)

	pthread_t tid;
	if (is_slam_mode)
		pthread_create(&tid, NULL, nd_map_mean_points_publisher, NULL);
	ros::Rate loop_rate(20);

#ifdef RECORD_NDMAP
	record_nd_map_as_txt();
#endif
	fprintf(stderr, "preparation for ndt scan matching is done\n");
	fprintf(stderr, "waiting for velodyne and odometry data\n");
	tf_init();
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	n.getParam("/ndt3d_with_odom/save_map", save_map);
	if (save_map)	save_nd_map(g_ndmap_name);

	return 0;
}
