#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ndt3d/NDTResult.h>
#include <Eigen/Dense>

#define POSE_BUFFER_SIZE (1000)

double record_interval = 3.0;
std::vector<geometry_msgs::PoseWithCovarianceStamped> filtered_poses;
std::vector<ndt3d::NDTResult> ndt_results;

bool get_ellipse_parameters(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose, double *a, double *b, double *yaw) {
	Eigen::Matrix2d A;
	A << pose->pose.covariance[0 * 6 + 0], pose->pose.covariance[0 * 6 + 1], pose->pose.covariance[1 * 6 + 0], pose->pose.covariance[1 * 6 + 1];
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(A);
	// eigen values could not be comupted
	if (es.info() != 0)
		return false;
	double ev1, ev2, chi2 = 9.21034;
	ev1 = es.eigenvalues()(0);
	ev2 = es.eigenvalues()(1);
	// eigen values are imaginary
	if (ev1 < 0.0 || ev2 < 0.0)
		return false;
	if (fabs(ev1) >= fabs(ev2)) {
		*a = sqrt(chi2 * ev1);
		*b = sqrt(chi2 * ev2);
		*yaw = atan2(es.eigenvectors()(0, 1), es.eigenvectors()(0, 0));
	} else {
		*a = sqrt(chi2 * ev2);
		*b = sqrt(chi2 * ev1);
		*yaw = atan2(es.eigenvectors()(1, 1), es.eigenvectors()(1, 0));
	}
	return true;
}

bool get_ellipse_parameters2(double m11, double m12, double m21, double m22, double *a, double *b, double *yaw) {
	Eigen::Matrix2d A;
	A << m11, m12, m21, m22;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(A);
	// eigen values could not be comupted
	if (es.info() != 0)
		return false;
	double ev1, ev2, chi2 = 9.21034;
	ev1 = es.eigenvalues()(0);
	ev2 = es.eigenvalues()(1);
	// eigen values are imaginary
	if (ev1 < 0.0 || ev2 < 0.0)
		return false;
	if (fabs(ev1) >= fabs(ev2)) {
		*a = sqrt(chi2 * ev1);
		*b = sqrt(chi2 * ev2);
		*yaw = atan2(es.eigenvectors()(0, 1), es.eigenvectors()(0, 0));
	} else {
		*a = sqrt(chi2 * ev2);
		*b = sqrt(chi2 * ev1);
		*yaw = atan2(es.eigenvectors()(1, 1), es.eigenvectors()(1, 0));
	}
	return true;
}

// this function is used first but it cannot work because velodyne and odometry data have a different time stamp.
// manual time synchronizer was implemented.
void sync_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &ndt_pose, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &filtered_pose) {
	// for first callback
	static bool is_first = true;
	static geometry_msgs::PoseWithCovarianceStamped old_filtered_pose;
	static FILE *fp_ndt_path, *fp_filtered_path, *fp_ndt_ellipse, *fp_filtered_ellipse;
	if (!fp_ndt_path)
		fp_ndt_path = fopen("/tmp/ndt_path", "w");
	if (!fp_filtered_path)
		fp_filtered_path = fopen("/tmp/filtered_path", "w");
	if (!fp_ndt_ellipse)
		fp_ndt_ellipse = fopen("/tmp/ndt_ellipse", "w");
	if (!fp_filtered_ellipse)
		fp_filtered_ellipse = fopen("/tmp/filtered_ellipse", "w");
	if (is_first) {
		old_filtered_pose = *filtered_pose;
		is_first = false;
		return;
	}

	// check interval
	double dx = filtered_pose->pose.pose.position.x - old_filtered_pose.pose.pose.position.x;
	double dy = filtered_pose->pose.pose.position.y - old_filtered_pose.pose.pose.position.y;
	double dz = filtered_pose->pose.pose.position.z - old_filtered_pose.pose.pose.position.z;
	double dl = sqrt(dx * dx + dy * dy + dz * dz);
	if (dl < record_interval)
		return;

	// rcompute ellipse parameters
	double ndt_a, ndt_b, ndt_yaw, filtered_a, filtered_b, filtered_yaw;
	if (!get_ellipse_parameters(ndt_pose, &ndt_a, &ndt_b, &ndt_yaw)) {
		ROS_ERROR("ellipse parameters of ndt were not computed");
		return;
	}
	if (!get_ellipse_parameters(filtered_pose, &filtered_a, &filtered_b, &filtered_yaw)) {
		ROS_ERROR("ellipse parameters of filtered ndt were not computed");
		return;
	}

	// record path
	fprintf(fp_ndt_path, "%lf %lf %lf\n", ndt_pose->pose.pose.position.x, ndt_pose->pose.pose.position.y, ndt_pose->pose.pose.position.z);
	fprintf(fp_filtered_path, "%lf %lf %lf\n", filtered_pose->pose.pose.position.x, filtered_pose->pose.pose.position.y, filtered_pose->pose.pose.position.z);
	fflush(fp_ndt_path);
	fflush(fp_filtered_path);

	// record ellipse
	double sx, sy, step = 5.0 * M_PI / 180.0;
	for (double yaw = 0.0; yaw < 2.0 * M_PI; yaw += step) {
		double tmpx = ndt_a * cos(yaw);
		double tmpy = ndt_b * sin(yaw);
		double x = tmpx * cos(ndt_yaw) - tmpy * sin(ndt_yaw) + ndt_pose->pose.pose.position.x;
		double y = tmpx * sin(ndt_yaw) + tmpy * cos(ndt_yaw) + ndt_pose->pose.pose.position.y;
		if (yaw == 0.0) {
			sx = x;
			sy = y;
		}
		fprintf(fp_ndt_ellipse, "%lf %lf\n", x, y);
	}
	fprintf(fp_ndt_ellipse, "%lf %lf\n", sx, sy);
	fprintf(fp_ndt_ellipse, "\n");
	fflush(fp_ndt_ellipse);
	for (double yaw = 0.0; yaw < 2.0 * M_PI; yaw += step) {
		double tmpx = ndt_a * cos(yaw);
		double tmpy = ndt_b * sin(yaw);
		double x = tmpx * cos(filtered_yaw) - tmpy * sin(filtered_yaw) + filtered_pose->pose.pose.position.x;
		double y = tmpx * sin(filtered_yaw) + tmpy * cos(filtered_yaw) + filtered_pose->pose.pose.position.y;
		if (yaw == 0.0) {
			sx = x;
			sy = y;
		}
		fprintf(fp_filtered_ellipse, "%lf %lf\n", x, y);
	}
	fprintf(fp_filtered_ellipse, "%lf %lf\n", sx, sy);
	fprintf(fp_filtered_ellipse, "\n");
	fflush(fp_filtered_ellipse);

	// save current pose
	old_filtered_pose = *filtered_pose;
}

void filtered_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
	filtered_poses.insert(filtered_poses.begin(), *msg);
	if (filtered_poses.size() >= POSE_BUFFER_SIZE)
		filtered_poses.resize(POSE_BUFFER_SIZE);
}

void ndt_result_callback(const ndt3d::NDTResult::ConstPtr &msg) {
	ndt_results.insert(ndt_results.begin(), *msg);
	if (ndt_results.size() >= POSE_BUFFER_SIZE)
		ndt_results.resize(POSE_BUFFER_SIZE);
}

int get_synchronized_filtered_pose(double time) {
	for (int i = 0; i < filtered_poses.size(); i++) {
		if (filtered_poses[i].header.stamp.toSec() < time)
			return i;
	}
	return -1;
}

int get_synchronized_ndt_result(double time) {
	for (int i = 0; i < ndt_results.size(); i++) {
		if (ndt_results[i].header.stamp.toSec() < time)
			return i;
	}
	return -1;
}

void ndt_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
	// for first callback
	static bool is_first = true;
	static geometry_msgs::PoseWithCovarianceStamped old_pose;
	static FILE *fp_ndt_path, *fp_filtered_path, *fp_ndt_ellipse, *fp_filtered_ellipse;
	if (!fp_ndt_path)
		fp_ndt_path = fopen("/tmp/ndt_path", "w");
	if (!fp_filtered_path)
		fp_filtered_path = fopen("/tmp/filtered_path", "w");
	if (!fp_ndt_ellipse)
		fp_ndt_ellipse = fopen("/tmp/ndt_ellipse", "w");
	if (!fp_filtered_ellipse)
		fp_filtered_ellipse = fopen("/tmp/filtered_ellipse", "w");
	if (is_first) {
		old_pose = *msg;
		is_first = false;
		return;
	}

	// check interval
	double dx = msg->pose.pose.position.x - old_pose.pose.pose.position.x;
	double dy = msg->pose.pose.position.y - old_pose.pose.pose.position.y;
	double dz = msg->pose.pose.position.z - old_pose.pose.pose.position.z;
	double dl = sqrt(dx * dx + dy * dy + dz * dz);
	if (dl < record_interval)
		return;

	// search synchronized filterd pose and ndt result
	int filtered_pose_index = get_synchronized_filtered_pose(msg->header.stamp.toSec());
	int ndt_result_index = get_synchronized_ndt_result(msg->header.stamp.toSec());
	if (filtered_pose_index < 0 || ndt_result_index < 0) {
		ROS_ERROR("no synchronized data");
		return;
	}
	geometry_msgs::PoseWithCovarianceStamped filtered_pose = filtered_poses[filtered_pose_index];
	ndt3d::NDTResult ndt_result = ndt_results[ndt_result_index];

	// compute ellipse parameters
	double ndt_a, ndt_b, ndt_t, filtered_a, filtered_b, filtered_t;
	if (!get_ellipse_parameters(msg, &ndt_a, &ndt_b, &ndt_t)) {
		ROS_ERROR("ellipse parameters of ndt were not computed");
		return;
	}
	if (!get_ellipse_parameters2(filtered_pose.pose.covariance[0 * 6 + 0], filtered_pose.pose.covariance[0 * 6 + 1], 
			filtered_pose.pose.covariance[1 * 6 + 0], filtered_pose.pose.covariance[1 * 6 + 1], &filtered_a, &filtered_b, &filtered_t)) {
		ROS_ERROR("ellipse parameters of filtered ndt were not computed");
		return;
	}

	// compute curvature
	static bool is_old_yaws = false;
	static double old_ndt_yaw, old_filtered_yaw;
	static geometry_msgs::PoseWithCovarianceStamped old_filtered_pose;
	double ndt_roll, ndt_pitch, ndt_yaw, filtered_roll, filtered_pitch, filtered_yaw, ndt_curvature, filtered_curvature;
	tf::Quaternion ndt_q(msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf::Matrix3x3 ndt_m(ndt_q);
	ndt_m.getRPY(ndt_roll, ndt_pitch, ndt_yaw);
	tf::Quaternion filtered_q(filtered_pose.pose.pose.orientation.x,
		filtered_pose.pose.pose.orientation.y,
		filtered_pose.pose.pose.orientation.z,
		filtered_pose.pose.pose.orientation.w);
	tf::Matrix3x3 filtered_m(filtered_q);
	filtered_m.getRPY(filtered_roll, filtered_pitch, filtered_yaw);
	if (!is_old_yaws) {
		ndt_curvature = filtered_curvature = 0.0;
		is_old_yaws = true;
	} else {
		double dx, dy, dl, dyaw;
		dx = msg->pose.pose.position.x - old_pose.pose.pose.position.x;
		dy = msg->pose.pose.position.y - old_pose.pose.pose.position.y;
		dl = sqrt(dx * dx + dy * dy);
		dyaw = ndt_yaw - old_ndt_yaw;
		if (dyaw < -M_PI)	dyaw += 2.0 * M_PI;
		if (dyaw > M_PI)	dyaw -= 2.0 * M_PI;
		ndt_curvature = dyaw / dl;
		dx = filtered_pose.pose.pose.position.x - old_filtered_pose.pose.pose.position.x;
		dy = filtered_pose.pose.pose.position.y - old_filtered_pose.pose.pose.position.y;
		dl = sqrt(dx * dx + dy * dy);
		dyaw = filtered_yaw - old_filtered_yaw;
		if (dyaw < -M_PI)	dyaw += 2.0 * M_PI;
		if (dyaw > M_PI)	dyaw -= 2.0 * M_PI;
		filtered_curvature = dyaw / dl;
	}
	old_ndt_yaw = ndt_yaw;
	old_filtered_yaw = filtered_yaw;
	old_filtered_pose = filtered_pose;

	// record path
	static int cnt;
	double ave_m_dist = ndt_result.error_distance / (double)ndt_result.scan_points_num;
	fprintf(fp_ndt_path, "%d %lf %lf %lf %lf %lf %lf\n", cnt, msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, ndt_yaw, ndt_curvature, ave_m_dist);
	fprintf(fp_filtered_path, "%d %lf %lf %lf %lf %lf\n", cnt, filtered_pose.pose.pose.position.x, filtered_pose.pose.pose.position.y, filtered_pose.pose.pose.position.z, 
		filtered_yaw, filtered_curvature);
	fflush(fp_ndt_path);
	fflush(fp_filtered_path);
	cnt++;

	// record ellipse
	double sx, sy, step = 5.0 * M_PI / 180.0;
	for (double t = 0.0; t < 2.0 * M_PI; t += step) {
		double tmpx = ndt_a * cos(t);
		double tmpy = ndt_b * sin(t);
		double x = tmpx * cos(ndt_t) - tmpy * sin(ndt_t) + msg->pose.pose.position.x;
		double y = tmpx * sin(ndt_t) + tmpy * cos(ndt_t) + msg->pose.pose.position.y;
		if (t == 0.0) {
			sx = x;
			sy = y;
		}
		fprintf(fp_ndt_ellipse, "%lf %lf\n", x, y);
	}
	fprintf(fp_ndt_ellipse, "%lf %lf\n", sx, sy);
	fprintf(fp_ndt_ellipse, "\n");
	fflush(fp_ndt_ellipse);
	for (double t = 0.0; t < 2.0 * M_PI; t += step) {
		double tmpx = filtered_a * cos(t);
		double tmpy = filtered_b * sin(t);
		double x = tmpx * cos(filtered_t) - tmpy * sin(filtered_t) + filtered_pose.pose.pose.position.x;
		double y = tmpx * sin(filtered_t) + tmpy * cos(filtered_t) + filtered_pose.pose.pose.position.y;
		if (t == 0.0) {
			sx = x;
			sy = y;
		}
		fprintf(fp_filtered_ellipse, "%lf %lf\n", x, y);
	}
	fprintf(fp_filtered_ellipse, "%lf %lf\n", sx, sy);
	fprintf(fp_filtered_ellipse, "\n");
	fflush(fp_filtered_ellipse);

	// save current pose
	old_pose = *msg;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "estimated_pose_logger");
	ros::NodeHandle nh("~");
/*
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> ndt_pose_sub(nh, "/ndt_current_pose_with_covariance", 1000);
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> filtered_pose_sub(nh, "/filtered_ndt_current_pose_with_covariance", 1000);
	message_filters::TimeSynchronizer<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> sync(ndt_pose_sub, filtered_pose_sub, 1);
	sync.registerCallback(boost::bind(&sync_pose_callback, _1, _2));
 */
	ros::Subscriber filtered_pose_sub = nh.subscribe("/filtered_ndt_current_pose_with_covariance", 1000, filtered_pose_callback);
	ros::Subscriber ndt_pose_sub = nh.subscribe("/ndt_current_pose_with_covariance", 1000, ndt_pose_callback);
	ros::Subscriber ndt_result_sub = nh.subscribe("/ndt_localization_result", 1000, ndt_result_callback);
	ros::spin();
	return 0;
}
