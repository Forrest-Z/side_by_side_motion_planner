#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <Eigen/Core>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include "safety_limiter/Status.h"

#include "astar.hpp"

namespace grid_local_planner
{
	class GridPlannerROS : public nav_core::BaseLocalPlanner 
	{
	private:
		ros::Publisher pub_plan_l;
		ros::Publisher pub_vel;
		ros::Publisher pub_subgoal;
		ros::Subscriber sub_safety_state_;
		safety_limiter::Status safety_state_;
		tf::TransformListener* tf_;
		costmap_2d::Costmap2DROS* costmap_ros_;
		costmap_2d::Costmap2D costmap_copy_;
		costmap_2d::Costmap2D *cm_;
		std::vector<geometry_msgs::PoseStamped> global_plan_;
		double goal_tolerance_xy_;
		double goal_tolerance_ang_;
		double w_lin_cost_;
		double w_lin_path_;
		double w_lin_yaw_;
		double w_lin_back_;
		double w_rot_base_;
		double w_rot_cost_base_;
		double w_rot_cost_;
		double vel_;
		int replan_cnt_;
		int replan_interval_;
		int sz_ang_;
		int search_dist_;
		int watchdog_;
		int watchdog_limit_;
		int no_motion_cnt_;
		ros::Time last_ret_true_;
		geometry_msgs::Pose goal_pose_;

		bool goal_;
		bool reach_;
		bool initialized_;
		bool last_retval_;
		bool strict_goal_pose_;
		bool temp_esc_;
		base_local_planner::OdometryHelperRos odom_helper_;
		tf::Stamped<tf::Pose> prev_pose_;
		
		astar::space c_path_;
		std::unique_ptr<int[]> arrivable_;
		astar::space dist_cache_;
		astar::astar search_;

		template <typename P>
			float dist2d(P &a, P &b)
			{
				return sqrtf(powf(a.getOrigin().x() - b.getOrigin().x(),2) + 
						powf(a.getOrigin().y() - b.getOrigin().y(),2));
			}
		template <typename P>
			float dist2d(geometry_msgs::Pose &a, P &b)
			{
				return sqrtf(powf(a.position.x-b.getOrigin().x(),2) + powf(a.position.y-b.getOrigin().y(),2));
			}
		template <typename P>
			float dist2d(P &b, geometry_msgs::Pose &a)
			{
				return sqrtf(powf(a.position.x-b.getOrigin().x(),2) + powf(a.position.y-b.getOrigin().y(),2));
			}
		float dist2d(geometry_msgs::Pose &a, geometry_msgs::Pose &b)
		{
			return sqrtf(powf(a.position.x-b.position.x,2) + powf(a.position.y-b.position.y,2));
		}

		void labeling(std::unique_ptr<int[]> &data, int width, int height)
		{
			int label = 1;
			std::map<int, int> label_merge;
			for(int y = 1; y < height - 1; y ++)
			{
				for(int x = 1; x < width - 1; x ++)
				{
					auto &p = data[y * width + x];
					auto &pl = data[y * width + x - 1];
					auto &pu = data[(y - 1) * width + x];
					if(p == 0) continue;

					int min = INT_MAX;
					if(pl != 0)
					{
						int a = pl;
						while(label_merge[a] != a) a = label_merge[a];
						min = std::min(a, min);
					}
					if(pu != 0)
					{
						int a = pu;
						while(label_merge[a] != a) a = label_merge[a];
						min = std::min(a, min);
					}
					if(min == INT_MAX)
					{
						min = ++label;
						label_merge[min] = min;
					}
					else
					{
						if(pl != min && pl != 0) label_merge[pl] = min;
						if(pu != min && pu != 0) label_merge[pu] = min;
					}
					p = min;
				}
			}
			{
				auto lc = label_merge;
				for(auto &l: label_merge)
					while(lc[l.second] != l.second)
						l.second = lc[l.second];
			}
			for(int y = 1; y < height - 1; y ++)
			{
				for(int x = 1; x < width - 1; x ++)
				{
					auto &l = data[y * width + x];
					if(l == 0) continue;
					l = label_merge[l];
				}
			}
		}

		bool ret_filter(bool ret)
		{
			if(ret)
			{
				last_ret_true_ = ros::Time::now();
				return true;
			}
			if((ros::Time::now() - last_ret_true_).toSec() < 5)
			{
				return true;
			}
			return false;
		}

		void findFeasiblePos(int &x, int &y,
				const std::pair<astar::uvo<>, astar::uvo<>> &rect, int max_cost = 128, int min_dist = 0)
		{
			auto cm = costmap_ros_->getCostmap();
			int range = 16;
			int x2 = x, y2 = y;
			int d2_min = lroundf(sqrtf(range*range));
			int min_cost = 256;
			for(int i = -range; i <= range; i ++)
			{
				for(int j = -range; j <= range; j ++)
				{
					if(x + i < rect.first.u_ || y + j < rect.first.v_ || 
							x + i >= rect.second.u_ || y + j >= rect.second.v_)
						continue;

					int d2 = lroundf(sqrtf(i*i + j*j));
					if(cm->getCost(x + i, y + j) < max_cost &&
							(d2 < d2_min || 
							 (d2 == d2_min && min_cost > cm->getCost(x + i, y + j))) && 
							d2 >= min_dist)
					{
						min_cost = cm->getCost(x + i, y + j);
						d2_min = d2;
						x2 = x + i;
						y2 = y + j;
					}
				}
			}
			x = x2;
			y = y2;
		}

	public:
		GridPlannerROS() :
			goal_tolerance_xy_(0.15),
			goal_tolerance_ang_(0.2),
			w_lin_cost_(3.0),
			w_lin_path_(0.25),
			w_lin_yaw_(0.5),
			w_lin_back_(0.45),
			w_rot_base_(5.0),
			w_rot_cost_base_(100.0),
			w_rot_cost_(3000.0),
			vel_(0.5),
			replan_cnt_(0),
			replan_interval_(20),
			search_dist_(5),
			goal_(false),
			reach_(false),
			initialized_(false),
			last_retval_(true),
			strict_goal_pose_(false),
			temp_esc_(false),
			odom_helper_("odom"),
			search_(search_dist_, 1, 0.1, 0.5)
		{
		}
		void initialize(std::string name, tf::TransformListener* tf,
				costmap_2d::Costmap2DROS* costmap_ros)
		{
			ros::NodeHandle private_nh("~/" + name);
			pub_plan_l = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
			pub_subgoal = private_nh.advertise<geometry_msgs::PoseStamped>("subgoal", 1);
			pub_vel = private_nh.advertise<std_msgs::Float32>("vel", 1);
			sub_safety_state_ = private_nh.subscribe("safety_state", 1, &GridPlannerROS::cbSafetyState, this);
			safety_state_.status = safety_limiter::Status::NORMAL;
			tf_ = tf;
			costmap_ros_ = costmap_ros;

			private_nh.param("goal_tolerance_xy", goal_tolerance_xy_, goal_tolerance_xy_);
			private_nh.param("goal_tolerance_ang", goal_tolerance_ang_, goal_tolerance_ang_);
			private_nh.param("w_lin_cost", w_lin_cost_, w_lin_cost_);
			private_nh.param("w_lin_path", w_lin_path_, w_lin_path_);
			private_nh.param("w_lin_yaw", w_lin_yaw_, w_lin_yaw_);
			private_nh.param("w_lin_back", w_lin_back_, w_lin_back_);
			private_nh.param("w_rot_base", w_rot_base_, w_rot_base_);
			private_nh.param("w_rot_cost_base", w_rot_cost_base_, w_rot_cost_base_);
			private_nh.param("w_rot_cost", w_rot_cost_, w_rot_cost_);
			private_nh.param("vel", vel_, vel_);
			private_nh.param("strict_goal_pose", strict_goal_pose_, strict_goal_pose_);
			private_nh.param("watchdog_limit", watchdog_limit_, 30);
			watchdog_ = watchdog_limit_;

			std::string odom_topic;
			if(private_nh.getParam("odom_topic", odom_topic))
			{
				odom_helper_.setOdomTopic(odom_topic);
			}

			sz_ang_ = 16;
			cm_ = costmap_ros_->getCostmap();
			search_.initialize(astar::uvo<>(cm_->getSizeInCellsX(), cm_->getSizeInCellsY(), sz_ang_));
			c_path_.reset(cm_->getSizeInCellsX(), cm_->getSizeInCellsY(), 1, 0);
			arrivable_.reset(new int[cm_->getSizeInCellsX() * cm_->getSizeInCellsY()]);

			dist_cache_.reset(search_dist_ * 2 + 1, search_dist_ * 2 + 1, 1, 0);
			for(int x = -search_dist_; x <= search_dist_; x ++)
			{
				for(int y = -search_dist_; y <= search_dist_; y ++)
				{
					dist_cache_.e(x + search_dist_, y + search_dist_, 0) = sqrtf(x*x + y*y);
				}
			}
			initialized_ = true;
			no_motion_cnt_ = 0;
			last_ret_true_ = ros::Time::now();
		}
		~GridPlannerROS()
		{
		}
		void cbSafetyState(const safety_limiter::StatusConstPtr &msg)
		{
			safety_state_ = *msg;
		}
		bool worldToMap(float x, float y, astar::uvo<> &p)
		{
			unsigned int su, sv;
			bool ret = cm_->worldToMap(x, y, su, sv);
			p.u_ = su;
			p.v_ = sv;
			return ret;
		}
		void mapToWorld(float mx, float my, float& wx, float& wy) const
		{
			wx = cm_->getOriginX() + (mx + 0.5) * cm_->getResolution();
			wy = cm_->getOriginY() + (my + 0.5) * cm_->getResolution();
		}
		bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
		{
			tf::Stamped<tf::Pose> current_pose;
			costmap_ros_->getRobotPose(current_pose);
			tf::Stamped<tf::Pose> robot_vel;
			odom_helper_.getRobotVel(robot_vel);

			float d_min = FLT_MAX;
			decltype(global_plan_.begin()) is;
			decltype(global_plan_.begin()) ie;

			// Find closest position on given path
			for(auto i = global_plan_.begin(); i != global_plan_.end(); i ++)
			{
				float d = dist2d(i->pose, current_pose);
				if(d < d_min)
				{
					d_min = d;
					is = i;
				}
			}
			// Current and previous position on the map
			astar::uvo<> s;
			astar::uvo<> s_diff;
			astar::uvo<> e;
			if(!worldToMap(
						current_pose.getOrigin().x(), 
						current_pose.getOrigin().y(), s))
			{
				ROS_WARN("Costmap not ready. current pose %0.3f %0.3f is out of costmap", current_pose.getOrigin().x(), current_pose.getOrigin().y());
				return ret_filter(last_retval_);
			}
			s.y_ = search_.yaw2i(tf::getYaw(current_pose.getRotation()));
			if(!worldToMap(
						prev_pose_.getOrigin().x(), 
						prev_pose_.getOrigin().y(), s_diff))
			{
				ROS_WARN("Costmap not ready. previous pose %0.3f %0.3f is out of costmap", prev_pose_.getOrigin().x(), prev_pose_.getOrigin().y());
				prev_pose_ = current_pose;
				return ret_filter(last_retval_);
			}
			s_diff.y_ = search_.yaw2i(tf::getYaw(prev_pose_.getRotation()));

			if(abs(s.u_ - c_path_.u_/2) > c_path_.u_/4 ||
				abs(s.v_ - c_path_.v_/2) > c_path_.v_/4)
			{
				ROS_WARN("Costmap not updated");
				return ret_filter(last_retval_);
			}
			s_diff = s - s_diff;
			auto rect = std::pair<astar::uvo<>, astar::uvo<>>(
							astar::uvo<>(1, 1, 0), 
							astar::uvo<>(c_path_.u_ - 1, c_path_.v_ - 1, 0));
			if(s_diff.u_ > 0)
				rect.first.u_ = s_diff.u_ + 1;
			else if(s_diff.u_ < 0)
				rect.second.u_ = c_path_.u_ + s_diff.u_ - 1;
			if(s_diff.v_ > 0)
				rect.first.v_ = s_diff.v_ + 1;
			else if(s_diff.v_ < 0)
				rect.second.v_ = c_path_.v_ + s_diff.v_ - 1;

			if(s_diff.u_ != 0 || s_diff.v_ != 0 || s_diff.y_ != 0)
				no_motion_cnt_ = 0;
			else
				no_motion_cnt_ ++;

			int cost_robot = cm_->getCost(s.u_, s.v_);

			int w = cm_->getSizeInCellsX();
			for(int v = 0; v < (int)cm_->getSizeInCellsY(); v ++)
			{
				for(int u = 0; u < (int)cm_->getSizeInCellsX(); u ++)
				{
					if(cm_->getCost(u, v) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
						arrivable_[u + v * w] = 0;
					else
						arrivable_[u + v * w] = -1;
					if(u < rect.first.u_ || v < rect.first.v_ || 
							u >= rect.second.u_ || v >= rect.second.v_)
						arrivable_[u + v * w] = 0;
				}
			}
			labeling(arrivable_, 
					(int)cm_->getSizeInCellsX(), (int)cm_->getSizeInCellsY());

			auto s2 = s;
			if(cost_robot >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
				findFeasiblePos(s2.u_, s2.v_, rect, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
			e = s;
			ie = is;
			bool near_goal = true;
			int label = arrivable_[s2.u_ + s2.v_*w];
			int unarrivable_cnt = 0;
			unsigned int xm_prev = 0, ym_prev = 0;
			for(auto i = is + 1; i != global_plan_.end(); i ++)
			{
				unsigned int xm, ym;
				if(cm_->worldToMap(i->pose.position.x, i->pose.position.y, xm, ym))
				{
					if((int)xm < rect.first.u_ || (int)ym < rect.first.v_ || 
							(int)xm >= rect.second.u_ || (int)ym >= rect.second.v_)
						continue;
					if(arrivable_[xm + ym*w] == label)
					{
						e.u_ = xm;
						e.v_ = ym;
						ie = i;
						unarrivable_cnt = 0;
					}
					else
					{
						if(xm_prev != xm || ym_prev != ym) unarrivable_cnt ++;
					}
					xm_prev = xm;
					ym_prev = ym;
				}
				else
				{
					near_goal = false;
					break;
				}
			}
			bool arrivable = true;
			if(unarrivable_cnt > 2)
			{
				ROS_WARN("Subgoal not arrivable for %d grids", unarrivable_cnt);
				arrivable = false;
			}
			bool temp_esc2(false);
			if(cm_->getCost(e.u_, e.v_) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
			{
				findFeasiblePos(e.u_, e.v_, rect, costmap_2d::INSCRIBED_INFLATED_OBSTACLE, 6);
				if(cm_->getCost(e.u_, e.v_) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
				{
					ROS_INFO("Failed to find feasible goal");
				}
			}
			geometry_msgs::Pose goal_pose_temp = goal_pose_;
			if(no_motion_cnt_ > watchdog_limit_ || 
					cost_robot >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
			{
				ROS_INFO("Temporal escape from collision");
				e = s;
				findFeasiblePos(e.u_, e.v_, rect, costmap_2d::INSCRIBED_INFLATED_OBSTACLE, 3);
				if(s.u_ == e.u_ && s.v_ == e.v_)
				{
					ROS_INFO("Failed to find feasible subgoal");
					last_retval_ = false;
					return ret_filter(false);
				}
				goal_pose_temp.orientation = tf::createQuaternionMsgFromYaw(
						atan2(s.v_ - e.v_, s.u_ - e.u_));
				e.y_ = search_.yaw2i(tf::getYaw(goal_pose_temp.orientation));
				temp_esc2 = true;
			}
			// navfn don't provide orientation of path node excepting last node
			bool no_goal_orientation = true;
			e.y_ = search_.yaw2i(tf::getYaw(ie->pose.orientation));
			if(near_goal)
			{
				no_goal_orientation = false;
			}
			if(near_goal &&
					dist2d(ie->pose, current_pose) < goal_tolerance_xy_ && 
					!reach_ && !temp_esc2 && !temp_esc_)
			{
				ROS_INFO("Reached");
				reach_ = true;
			}
			else if(reach_ && !temp_esc2)
			{
				if(no_motion_cnt_ > watchdog_limit_)
				{
					last_retval_ = false;
					return ret_filter(false);
				}
				worldToMap(is->pose.position.x, is->pose.position.y, s);
				s.y_ = search_.yaw2i(tf::getYaw(is->pose.orientation));
				e = s;
				float ang_diff = tf::getYaw(ie->pose.orientation) - tf::getYaw(current_pose.getRotation());
				while(ang_diff > M_PI) ang_diff -= M_PI * 2;
				while(ang_diff < -M_PI) ang_diff += M_PI * 2;
				if(fabs(ang_diff) < goal_tolerance_ang_ && reach_)
				{
					if(!goal_) ROS_INFO("Oriented");
					goal_ = true;
					last_retval_ = true;
					return ret_filter(true);
				}
				{
					nav_msgs::Path path;
					path.header.stamp = ros::Time::now();
					path.header.frame_id = costmap_ros_->getGlobalFrameID();
					geometry_msgs::PoseStamped ps;
					ps.header = path.header;
					ps.header.seq = 0;
					if(strict_goal_pose_)
						ps.pose = goal_pose_temp;
					else
						ps.pose = ie->pose;
					float yaw = tf::getYaw(ps.pose.orientation);
					ps.pose.position.x -= cos(yaw) * 0.04;
					ps.pose.position.y -= sin(yaw) * 0.04;
					path.poses.push_back(ps);
					ps.pose.position.x += cos(yaw) * 0.02;
					ps.pose.position.y += sin(yaw) * 0.02;
					path.poses.push_back(ps);
					ps.pose.position.x += cos(yaw) * 0.02;
					ps.pose.position.y += sin(yaw) * 0.02;
					path.poses.push_back(ps);
					pub_plan_l.publish(path);
				}
				last_retval_ = true;
				return ret_filter(true);
			}
			if(s.u_ == e.u_ && s.v_ == e.v_ && !reach_)
			{
				ROS_WARN("Failed to search path: Oops! I am in Rock!");
				last_retval_ = false;
				
				auto e_prev = e;
				findFeasiblePos(e.u_, e.v_, rect, costmap_2d::INSCRIBED_INFLATED_OBSTACLE, 6);
				if(e == e_prev)
				{
					ROS_WARN("Impossible to escape!");
					last_retval_ = true;
					return ret_filter(false);
				}
				goal_pose_temp.orientation = tf::createQuaternionMsgFromYaw(atan2(e_prev.v_ - e.v_, e_prev.u_ - e.u_));
				e.y_ = search_.yaw2i(tf::getYaw(goal_pose_temp.orientation));

				ROS_INFO("Temporal escape");
				temp_esc2 = true;
			}
			replan_cnt_ ++;
			if(safety_state_.status == safety_limiter::Status::DECELERATING)
				watchdog_ --;
			else
				watchdog_ = watchdog_limit_;
			if(watchdog_ <= 0)
			{
				if(watchdog_ == 0) ROS_WARN("Safety stop");
				last_retval_ = false;
				return ret_filter(false);
			}

			if((robot_vel.getOrigin().x() < -0.01 || temp_esc_) && replan_cnt_ < replan_interval_) 
				return ret_filter(last_retval_);
			replan_cnt_ = 0;

			temp_esc_ = temp_esc2;

			for(int x = 0; x < c_path_.u_; x ++)
			{
				for(int y = 0; y < c_path_.v_; y ++)
				{
					float d_min = FLT_MAX;
					for(auto i = is; i != ie + 1; i ++)
					{
						i ++; if(i == ie + 1) break;
						i ++; if(i == ie + 1) break;

						double wx, wy;
						cm_->mapToWorld(x, y, wx, wy);
						geometry_msgs::Pose p;
						p.position.x = wx;
						p.position.y = wy;
						float d = dist2d(i->pose, p);
						if(d < d_min) d_min = d;
					}
					c_path_.e(x,y,0) = d_min * w_lin_path_;
				}
			}
			std::vector<astar::uvo<float>> path_grid;

			{
				geometry_msgs::PoseStamped ps;
				float x, y;
				mapToWorld(e.u_, e.v_, x, y);
				ps.header.stamp = ros::Time::now();
				ps.header.frame_id = costmap_ros_->getGlobalFrameID();
				ps.pose.position.x = x;
				ps.pose.position.y = y;
				ps.pose.position.z = 0;
				if(near_goal)
				{
					ps.pose.orientation = goal_pose_temp.orientation;
				}
				else
				{
					float yaw = atan2(ie->pose.position.y - (ie-1)->pose.position.y,
							ie->pose.position.x - (ie-1)->pose.position.x);
					ps.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
				}
				pub_subgoal.publish(ps);
			}
			
			float w_lin_yaw_p_ang = w_lin_yaw_ / (float)sz_ang_;
			bool timedout(false);
			if(!search_.search(s, e, 
						[&](astar::uvo<> &pos, astar::uvo<> &diff, bool forward)->float
						{
						auto next = pos + diff;
						int num = std::max(std::max(diff.u_, diff.v_), 1);
						int c_int = cm_->getCost(next.u_, next.v_);
						float c = 0;
						int cost_prev = cm_->getCost(pos.u_, pos.v_);
						for(int i = 1; i <= num; i ++)
						{
							int u = pos.u_ + diff.u_ * i / num;
							int v = pos.v_ + diff.v_ * i / num;
							if(u == pos.u_ && v == pos.v_) continue;
							int cost = cm_->getCost(u, v);
							if(cost > c_int) c_int = cost;
						}
						if(c_int >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
						{
							//if(!temp_esc_) return FLT_MAX/2;
							if(cost_prev > c_int) return 100 * c_int;
							return FLT_MAX/2;

						}
						//else if(c_int >= 128) return c = 1000 + 100 * c_int / 128.0;
						else c = c_int / 128.0;

						if(diff.u_ == 0 && diff.v_ == 0)
						{
							float d = w_rot_base_;
							if(c >= 1.0) return FLT_MAX/2;
							if(c > 0.0) d = w_rot_cost_base_ + c * w_rot_cost_;
							//d += 10 * fabs(diff.y_) / (float)sz_ang_;
							return d;
						}
						float len = dist_cache_.e(
								diff.u_ + search_dist_, 
								diff.v_ + search_dist_, 0);
						float d = 1;
						d += w_lin_cost_ * c;
						d += c_path_.e(next);
						d += w_lin_yaw_p_ang * fabs(diff.y_);
						if(!forward) d += w_lin_back_;
						if(!forward && c_int >= 128)
						{
							//if(!temp_esc_) return FLT_MAX/2;
							if(cost_prev > c_int) return 100 * c_int;
							return FLT_MAX/2;
						}

						d *= len;
						return d;
						}, no_goal_orientation, false, 
							std::pair<astar::uvo<>, astar::uvo<>>(
									astar::uvo<>(0, 0, 0), 
									astar::uvo<>(c_path_.u_, c_path_.v_, 0)),
							path_grid,
							[&](void)->bool
							{
								ROS_WARN("Stopping since path search takes long time");
								nav_msgs::Path path;
								path.header.stamp = ros::Time::now();
								path.header.frame_id = costmap_ros_->getGlobalFrameID();
								pub_plan_l.publish(path);

								timedout = true;

								return false;
							}, 0.25,
							[&](astar::space &estim)->void
							{
								float rot_cost = std::min(w_lin_yaw_p_ang, (float)w_rot_cost_);
								for(int x = 0; x < estim.u_; x ++)
								{
									for(int y = 0; y < estim.v_; y ++)
									{
										float d = sqrtf((x - e.u_)*(x - e.u_) + (y - e.v_)*(y - e.v_));
										d += (powf(c_path_.e(x, y, 0), 2) / w_lin_path_) / 2; // = w_lin_path_ * path_dist^2 / 2
										for(int o = 0; o < estim.y_; o ++)
										{
											float yaw_diff = estim.cyclic(e.y_ - o);
											estim.e(x,y,o) = d
												+ rot_cost * fabs(yaw_diff);
										}
									}
								}
							}))
			{
				ROS_WARN("Failed to search path: I am cooped up!");
				last_retval_ = false;
				return ret_filter(false);
			}

			nav_msgs::Path path;
			path.header.stamp = ros::Time::now();
			path.header.frame_id = costmap_ros_->getGlobalFrameID();
			float path_length = 0;
			for(auto &p: path_grid)
			{
				geometry_msgs::PoseStamped ps;
				float x, y;
				mapToWorld(p.u_, p.v_, x, y);
				ps.header = path.header;
				ps.pose.position.x = x;
				ps.pose.position.y = y;
				ps.pose.position.z = 0;
				ps.pose.orientation = tf::createQuaternionMsgFromYaw(p.y_);
				path.poses.push_back(ps);
			}
			auto i_prev = path.poses.begin();
			for(auto i = path.poses.begin(); i != path.poses.end(); i ++)
			{
				if(i != i_prev)
				{
					path_length += sqrtf(
							powf(i->pose.position.x - i_prev->pose.position.x, 2) +
							powf(i->pose.position.y - i_prev->pose.position.y, 2));
				}
				i_prev = i;
			}
			bool forward(false);
			if(path_grid.size() > 1)
			{
				float dir_diff = path_grid[0].y_ - 
					atan2(path_grid[1].v_ - path_grid[0].v_, path_grid[1].u_ - path_grid[0].u_);
				if(cos(dir_diff) > 0) forward = true;
			}
			prev_pose_ = current_pose;
			std_msgs::Float32 f;
			f.data = vel_ * 
				(1.00 - 
					std::min(
						(std::max(cost_robot, 64) - 64) / 128.0, 
						0.9)
					);
			if(!forward) f.data *= 0.7;
			pub_vel.publish(f);
			pub_plan_l.publish(path);

			if(temp_esc_ || !arrivable)
			{
				last_retval_ = false;
				return ret_filter(false);
			}
			last_retval_ = true;
			return ret_filter(true);
		}
		bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
		{
			ROS_INFO("Global plan applied");
			global_plan_ = orig_global_plan;
			goal_ = false;
			reach_ = false;
			replan_cnt_ = replan_interval_;
			goal_pose_ = (global_plan_.end()-1)->pose;
			return true;
		}
		bool isGoalReached()
		{
			return goal_;
		}
		bool isInitialized()
		{
			return initialized_;
		}
	};
};

PLUGINLIB_EXPORT_CLASS(grid_local_planner::GridPlannerROS, nav_core::BaseLocalPlanner)

