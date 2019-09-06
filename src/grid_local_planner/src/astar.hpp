#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cfloat>
#include <list>
#include <map>
#include <unordered_map>
#include <iostream>
#include <queue>
#include <chrono>

#include "filter.hpp"


namespace astar
{
	template <class T>
		class reservable_priority_queue: public std::priority_queue<T>
	{
	public:
		typedef typename std::priority_queue<T>::size_type size_type;
		reservable_priority_queue(size_type capacity = 0)
		{
			reserve(capacity);
		};
		void reserve(size_type capacity)
		{
			this->c.reserve(capacity);
		} 
		size_type capacity() const
		{
			return this->c.capacity();
		}
		void clear()
		{
			this->c.clear();
		}	   
	};

	template<typename T = int> class uvo
	{
	public:
		T u_;
		T v_;
		T y_;
		uvo(T u, T v, T y)
		{
			u_ = u; v_ = v; y_ = y;
		}
		uvo()
		{
			u_ = v_ = y_ = 0;
		}
		bool operator==(const uvo& b) const
		{
			return this->u_ == b.u_ && this->v_ == b.v_ && this->y_ == b.y_;
		}
		bool operator!=(const uvo& b) const
		{
			return !(*this == b);
		}
		operator uvo<int> () const
		{
			uvo<int> out;
			out.u_ = this->u_;
			out.v_ = this->v_;
			out.y_ = this->y_;
			return out;
		}
		operator uvo<float> () const
		{
			uvo<float> out;
			out.u_ = this->u_;
			out.v_ = this->v_;
			out.y_ = this->y_;
			return out;
		}
		uvo operator+(const uvo& b) const
		{
			uvo out;
			out.u_ = this->u_ + b.u_;
			out.v_ = this->v_ + b.v_;
			out.y_ = this->y_ + b.y_;
			return out;
		}
		uvo operator-(const uvo& b) const
		{
			uvo out;
			out.u_ = this->u_ - b.u_;
			out.v_ = this->v_ - b.v_;
			out.y_ = this->y_ - b.y_;
			return out;
		}
		bool operator<(const uvo& b) const
		{
			if(this->u_ == b.u_)
			{
				if(this->v_ == b.v_)
				{
					return this->y_ < b.y_;
				}
				return this->v_ < b.v_;
			}
			return this->u_ < b.u_;
		}
	};
	struct uvo_hash
	{
		typedef std::size_t result_type;
		std::size_t rotl(const std::size_t x, const std::size_t n) const
		{
			  return (x << n) | (x >> ((8 * sizeof(std::size_t)) - n));
		}
		std::size_t operator()(const uvo<>& key) const
		{
			return rotl(key.u_, 8 * sizeof(std::size_t) / 3) ^
				rotl(key.v_, 8 * sizeof(std::size_t) / 3) ^
				rotl(key.y_, 8 * sizeof(std::size_t) / 3);
		}
	};
	class pq
	{
	public:
		float p_;
		float p_raw_;
		uvo<> d_;
		pq()
		{
			p_ = 0;
			p_raw_ = 0;
		}
		pq(float p, float p_raw, uvo<> d)
		{
			p_ = p;
			p_raw_ = p_raw;
			d_ = d;
		}
		bool operator<(const pq& b) const
		{
			// smaller first
			return this->p_ > b.p_;
		}
	};
	class space
	{
	public:
		std::unique_ptr<float[]> a_;
		int u_;
		int v_;
		int y_;
		space()
		{
		}
		space(int u, int v, int y, int zero)
		{
			reset(u, v, y, zero);
		}
		space &operator=(space &a)
		{
			u_ = a.u_;
			v_ = a.v_;
			y_ = a.y_;
			a_.reset(new float[u_*v_*y_]);
			for(int i = 0; i < u_*v_*y_; i ++) a_[i] = a.a_[i];
			return *this;
		}
		inline void reset(int u, int v, int y, float zero)
		{
			u_ = u;
			v_ = v;
			y_ = y;
			a_.reset(new float[u*v*y]);
			for(int i = 0; i < u*v*y; i ++) a_[i] = zero;
		}
		inline int cyclic(int y)
		{
			y = y % y_;
			while(y < 0) y += y_;
			return y;
		}
		inline int yaw2i(float yaw)
		{
			return cyclic(lround(y_ * yaw / (2.0 * M_PI)));
		}
		inline float i2yaw(int i)
		{
			return 2.0 * M_PI * i / y_;
		}
		inline bool isValid(int u, int v, int y)
		{
			if((unsigned int)u >= (unsigned int)u_) return false;
			if((unsigned int)v >= (unsigned int)v_) return false;
			return true;
		}
		inline bool isValid(const uvo<> &p)
		{
			return isValid(p.u_, p.v_, p.y_);
		}
		inline bool bound(int &u, int &v, int &y,
				const std::pair<uvo<>, uvo<>> &rect)
		{
			bool ret = true;
			if(u < rect.first.u_)
			{
				u = rect.first.u_;
				ret = false;
			}
			else if(u >= rect.second.u_)
			{
				u = rect.second.u_ - 1;
				ret = false;
			}
			if(v < rect.first.v_)
			{
				v = rect.first.v_;
				ret = false;
			}
			else if(v >= rect.second.v_)
			{
				v = rect.second.v_ - 1;
				ret = false;
			}
			y = cyclic(y);
			return ret;
		}
		inline bool bound(int &u, int &v, int &y)
		{
			return bound(u, v, y, std::pair<uvo<>, uvo<>>(
						uvo<>(0, 0, 0), uvo<>(u_, v_, y_)
						));
		}
		inline bool bound(uvo<> &a)
		{
			return bound(a.u_, a.v_, a.y_);
		}
		inline bool bound(uvo<> &a,
				const std::pair<uvo<>, uvo<>> &rect)
		{
			return bound(a.u_, a.v_, a.y_, rect);
		}
		inline float &e(int u, int v, int y)
		{
			bound(u, v, y);
			return a_[y + y_ * (v + v_ * (u))];
		}
		inline float &e(uvo<> p)
		{
			bound(p);
			return e(p.u_, p.v_, p.y_);
		}
	};
	class astar
	{
	private:
		int sz_ang_;
		int search_dist_;
		int search_ang_;
		float interval_;
		float filter_step_;

		space sp_;
		space c_estim_;
		space atan_;

		std::unordered_map<uvo<>, uvo<>, uvo_hash> parents;
		reservable_priority_queue<pq> open;

	public:
		inline int yaw2i(float yaw)
		{
			return sp_.yaw2i(yaw);
		}
		inline float i2yaw(int i)
		{
			return sp_.i2yaw(i);
		}
		astar(int search_dist, int search_ang, float interval, float filter_step) :
			search_dist_(search_dist),
			search_ang_(search_ang),
			interval_(interval),
			filter_step_(filter_step/interval)
		{
		}
		void initialize(uvo<> size)
		{
			sz_ang_ = size.y_;
			sp_.reset(size.u_, size.v_, sz_ang_, FLT_MAX/2);
			c_estim_.reset(size.u_, size.v_, sz_ang_, 0);
			open.reserve(size.u_ * size.v_ * sz_ang_);
			parents.reserve(size.u_ * size.v_ * sz_ang_);

			atan_.reset(search_dist_ * 2 + 1, search_dist_ * 2 + 1, 1, 0);
			for(int x = -search_dist_; x <= search_dist_; x ++)
			{
				for(int y = -search_dist_; y <= search_dist_; y ++)
				{
					atan_.e(x + search_dist_, y + search_dist_, 0) = yaw2i(atan2(y, x));
				}
			}
		}
		bool search(uvo<> s, uvo<> e, 
				std::function<float(uvo<>&, uvo<>&, bool)> cost, 
				bool no_goal_orientation, 
				bool find_feasible_goal,
				const std::pair<uvo<>, uvo<>> &rect,
				std::vector<uvo<float>> &path,
				std::function<bool(void)> timeout_func = nullptr,
				double timeout = -1.0,
				std::function<void(space&)> estimation_func = nullptr)
		{
			auto t1 = std::chrono::system_clock::now();
			std::chrono::microseconds timeout_d(uint64_t(timeout * 1000000.0));
			if(estimation_func)
			{
				estimation_func(c_estim_);
			}
			else
			{
				for(int x = 0; x < sp_.u_; x ++)
				{
					for(int y = 0; y < sp_.v_; y ++)
					{
						float d = sqrtf((x - e.u_)*(x - e.u_) + (y - e.v_)*(y - e.v_));
						for(int o = 0; o < sp_.y_; o ++)
						{
							c_estim_.e(x,y,o) = d;
						}
					}
				}
			}
			{
				for(int x = 0; x < sp_.u_; x ++)
				{
					for(int y = 0; y < sp_.v_; y ++)
					{
						for(int o = 0; o < sp_.y_; o ++)
						{
							sp_.e(x,y,o) = FLT_MAX / 2;
						}
					}
				}
			}
			open.clear();
			parents.clear();
			sp_.e(s) = 1;
			open.push(pq(1 + c_estim_.e(s), 1, s));
			bool timedout = false;
			while(true)
			{
				auto t2 = std::chrono::system_clock::now();
				auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
				if(timeout > 0 && elapsed > timeout_d && !timedout)
				{
					timedout = true;
					if(timeout_func)
					{
						if(timeout_func()) return false;
					}
					else
					{
						return false;
					}
				}
				bool goal_found = false;
				if(open.size() < 1)
				{
					//std::cerr << "not connected" << std::endl;
					if(find_feasible_goal)
					{
						float min = FLT_MAX/2;
						for(int x = 0; x < sp_.u_; x ++)
						{
							for(int y = 0; y < sp_.v_; y ++)
							{
								for(int o = 0; o < sp_.y_; o ++)
								{
									if(sp_.e(x, y, o)*0.1 + c_estim_.e(x, y, o) < min)
									{
										e.u_ = x;
										e.v_ = y;
										e.y_ = o;
										min = sp_.e(e)*0.1 + c_estim_.e(e);
									}
								}
							}
						}
						if(e == s) return false;
						break;
					}
					return false;
				}
				auto p = open.top();
				uvo<> pos = p.d_;
				float c_min = p.p_raw_;
				open.pop();
				for(int xp = -search_dist_; xp <= search_dist_; xp ++)
				{
					for(int yp = -search_dist_; yp <= search_dist_; yp ++)
					{
						if(xp*xp + yp*yp > search_dist_*search_dist_) continue;
						for(int op = -sp_.y_/2; op < sp_.y_ - sp_.y_/2; op ++)
						{
							if(xp == 0 && yp == 0 && op == 0) continue;
							uvo<> diff(xp, yp, op);
							uvo<> next = pos + diff;
							if(!sp_.bound(next, rect)) continue;

							bool forward = true;
							if(!(xp == 0 && yp == 0))
							{
								if(abs(op) > search_ang_) continue;
								float yaw = atan_.e(
										xp + search_dist_, 
										yp + search_dist_, 0);
								int yaw_diff = yaw - next.y_;
								if(yaw_diff < -sz_ang_/2) yaw_diff += sz_ang_;
								else if(yaw_diff > sz_ang_/2) yaw_diff -= sz_ang_;

								if(abs(yaw_diff) <= search_ang_)
									forward = true;
								else if(abs(yaw_diff) >= sz_ang_/2 - search_ang_)
									forward = false;
								else continue;
							}

							float c = cost(pos, diff, forward);
							if(c >= FLT_MAX/2) continue;

							if(sp_.e(next) > c_min + c) 
							{
								sp_.e(next) = c_min + c;
								parents[next] = pos;
								open.push(pq(c_min + c + c_estim_.e(next), 
											c_min + c, next));
								if(next.u_ == e.u_ && next.v_ == e.v_ && 
										(next.y_ == e.y_ || no_goal_orientation))
								{
									goal_found = true;
								}
							}
						}
						if(goal_found) break;
					}
					if(goal_found) break;
				}
				if(goal_found) break;
			}
			if(no_goal_orientation)
			{
				float min = FLT_MAX;
				for(int i = 0; i < sz_ang_; i ++)
				{
					if(min > sp_.e(e.u_, e.v_, i))
					{
						min = sp_.e(e.u_, e.v_, i);
						e.y_ = i;
					}
				}
			}

			std::list<uvo<>> plan;
			{
				uvo<> n = e;
				while(true)
				{
					plan.push_front(n);
					sp_.bound(n);
					if(n == s) break;
					if(parents.find(n) == parents.end())
					{
						//std::cerr << "A* result invalid" << std::endl;
						return false;
					}
					n = parents[n];
				}
			}
			if(plan.size() > 0)
			{
				plan.front().y_ = s.y_;
				plan.back().y_ = e.y_;
				auto p_prev = plan.front();
				filter lpf_x(FILTER_LPF, filter_step_, p_prev.u_);
				filter lpf_y(FILTER_LPF, filter_step_, p_prev.v_);
				filter lpf_yx(FILTER_LPF, filter_step_, cos(i2yaw(p_prev.y_)));
				filter lpf_yy(FILTER_LPF, filter_step_, sin(i2yaw(p_prev.y_)));
				for(auto &p: plan)
				{
					if(p != p_prev)
					{
						// Interpolate
						float dx = (float)p.u_ - p_prev.u_;
						float dy = (float)p.v_ - p_prev.v_;
						int num = ceilf(sqrtf(dx*dx + dy*dy) / interval_) + 1;
						dx /= (float)num;
						dy /= (float)num;

						float yawx = cos(i2yaw(p_prev.y_));
						float yawy = sin(i2yaw(p_prev.y_));
						float dyawx = (yawx - cos(i2yaw(p.y_))) / (float)num;
						float dyawy = (yawy - sin(i2yaw(p.y_))) / (float)num;

						uvo<float> pflt = p_prev;
						pflt.y_ = i2yaw(pflt.y_);
						for(int j = 0; j < num; j ++)
						{
							path.push_back(
									uvo<float>(
										lpf_x.in(pflt.u_), 
										lpf_y.in(pflt.v_), 
										pflt.y_));
							pflt.u_ += dx;
							pflt.v_ += dy;
							yawx += dyawx;
							yawy += dyawy;
							if(sz_ang_ > 0)
								pflt.y_ = atan2(lpf_yy.in(yawy), lpf_yx.in(yawx));
							else
								pflt.y_ = 0;
						}
					}
					p_prev = p;
				}
				uvo<float> pflt = p_prev;
				path.push_back(pflt);
			}
			else
			{
				return false;
			}

			return true;
		}
	};
};


