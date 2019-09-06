#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_list_macros.h>

#include "astar.hpp"

namespace grid_local_planner
{
	class GlobalPlanner : public nav_core::BaseGlobalPlanner 
	{
	private:
		ros::Publisher pub_plan_g;
		costmap_2d::Costmap2DROS* costmap_ros_;
		std::vector<geometry_msgs::PoseStamped> global_plan_;
		int search_dist_;
		bool find_approx_goal_;
		
		astar::space c_path_;
		astar::space dist_cache_;
		astar::astar search_;

	public:
		~GlobalPlanner()
		{
		}
		void initialize(std::string name, 
				costmap_2d::Costmap2DROS* costmap_ros)
		{
			ros::NodeHandle private_nh("~/" + name);
			pub_plan_g = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
			costmap_ros_ = costmap_ros;
			
			private_nh.param("find_approx_goal", find_approx_goal_, false);

			auto cm = costmap_ros_->getCostmap();
			search_.initialize(astar::uvo<>(cm->getSizeInCellsX(), cm->getSizeInCellsY(), 1));
			dist_cache_.reset(search_dist_ * 2 + 1, search_dist_ * 2 + 1, 1, 0);
			for(int x = -search_dist_; x <= search_dist_; x ++)
			{
				for(int y = -search_dist_; y <= search_dist_; y ++)
				{
					dist_cache_.e(x + search_dist_, y + search_dist_, 0) = sqrtf(x*x + y*y);
				}
			}
		}
		GlobalPlanner() :
			search_dist_(4),
			search_(search_dist_, 0, 0.2, 0.5)
		{
		}
		GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
			search_dist_(4),
			search_(search_dist_, 0, 0.2, 0.5)
		{
			initialize(name, costmap_ros);
		}
		bool worldToMap(float x, float y, astar::uvo<> &p)
		{
			unsigned int su, sv;
			bool ret = costmap_ros_->getCostmap()->worldToMap(x, y, su, sv);
			p.u_ = su;
			p.v_ = sv;
			return ret;
		}
		void findFeasiblePos(int &x, int &y)
		{
			auto cm = costmap_ros_->getCostmap();
			int range = 16;
			int x2 = x, y2 = y;
			int d2_min = (range + 1) * (range + 1);
			for(int i = -range; i <= range; i ++)
			{
				for(int j = -range; j <= range; j ++)
				{
					if(x + i < 0 || y + j < 0 || 
							x + i >= (int)cm->getSizeInCellsX() || 
							y + j >= (int)cm->getSizeInCellsY())
						continue;
					int d2 = i*i + j*j;
					if((int)cm->getCost(x + i, y + j) < 128 &&
							d2 < d2_min)
					{
						d2_min = d2;
						x2 = x + i;
						y2 = y + j;
					}
				}
			}
			x = x2;
			y = y2;
		}
		void mapToWorld(float mx, float my, float& wx, float& wy) const
		{
			auto cm = costmap_ros_->getCostmap();
			wx = cm->getOriginX() + (mx + 0.5) * cm->getResolution();
			wy = cm->getOriginY() + (my + 0.5) * cm->getResolution();
		}
		bool makePlan(const geometry_msgs::PoseStamped& start, 
				const geometry_msgs::PoseStamped& goal, 
				std::vector<geometry_msgs::PoseStamped>& plan)
		{
			auto cm = costmap_ros_->getCostmap();

			astar::uvo<> s;
			astar::uvo<> e;
			if(!worldToMap(start.pose.position.x, start.pose.position.y , s))
			{
				ROS_WARN("Costmap not ready");
				return true;
			}
			s.y_ = 0;
			if(!worldToMap(goal.pose.position.x, goal.pose.position.y , e))
			{
				ROS_WARN("Costmap not ready");
				return true;
			}
			e.y_ = 0;
			if(cm->getCost(s.u_, s.v_) >= 128)
			{
				ROS_WARN("Start %d,%d", s.u_, s.v_);
				findFeasiblePos(s.u_, s.v_);
				ROS_WARN("  Moved to %d,%d", s.u_, s.v_);
				if(cm->getCost(s.u_, s.v_) >= 128)
				{
					return false;
				}
			}
			if(cm->getCost(e.u_, e.v_) >= 128)
			{
				ROS_WARN("Goal %d,%d", e.u_, e.v_);
				findFeasiblePos(e.u_, e.v_);
				ROS_WARN("  Moved to %d,%d", e.u_, e.v_);
				if(cm->getCost(e.u_, e.v_) >= 128)
				{
					return false;
				}
			}

			std::vector<astar::uvo<float>> path_grid;
			if(!search_.search(s, e, 
					[&](astar::uvo<> &pos, astar::uvo<> &diff, bool forward)->float
					{
						int num = std::max(std::max(diff.u_,diff.v_), 1);
						int c_int = 0;
						float c = 0;
						for(int i = 0; i < num; i ++)
						{
							int u = pos.u_ + diff.u_ * i / num;
							int v = pos.v_ + diff.v_ * i / num;
							int cost = cm->getCost(u, v);
							if(cost > c_int) c_int = cost;
						}
						if(c_int >= 128 + 64) return FLT_MAX;
						else if(c_int >= 128) return c = 1000 + 100 * c_int / 128.0;
						else c = c_int / 128.0;
						float len = dist_cache_.e(
										diff.u_ + search_dist_, 
										diff.v_ + search_dist_, 0);

						return len + c * 3.0;
					}, false, find_approx_goal_, 
						std::pair<astar::uvo<>, astar::uvo<>>(
							astar::uvo<>(0, 0, 0), 
							astar::uvo<>(cm->getSizeInCellsX(), cm->getSizeInCellsY(), 0)),
						path_grid))
			{
				ROS_WARN("Failed to search path");
				return false;
			}
			path_grid.back().y_ = tf::getYaw(goal.pose.orientation);

			nav_msgs::Path path;
			path.header.stamp = ros::Time::now();
			path.header.frame_id = costmap_ros_->getGlobalFrameID();
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
				plan.push_back(ps);
			}
			pub_plan_g.publish(path);

			return true;
		}
	};
};

PLUGINLIB_EXPORT_CLASS(grid_local_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

