/*********************************************************************
 * Copyright (C) 2015- Future Robotics Technology Center (fuRo),
 *                     Chiba Institute of Technology.
 *                     All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * fuRo ("Confidential Information"). You shall not disclose such
 * Confidential Information and shall use it only in accordance with
 * the terms of the license agreement you entered into with fuRo.
 *
 * @author Yoshitaka Hara
 *********************************************************************/

#ifndef RONA_UTIL_HPP
#define RONA_UTIL_HPP

// NOTE: librona からの一部移植

namespace rona
{

// math.hpp
//! 周期変数 val を [0, +max) の左閉右開区間に収める
inline double wrapToZeroMax(double val, double max)
{
    assert(max > 0.0);
    return std::fmod(max + std::fmod(val, max), max);
}
//! 周期変数 val を [-max, +max) の左閉右開区間に収める
inline double wrapToNegPos(double val, double max)
{
    return wrapToZeroMax(val + max, 2.0 * max) - max;
}

// angle.hpp
//! 角度 [rad] を [-pi, +pi) の値域に収める
inline double wrapToNegPosPi(double ang_rad)
{
    return wrapToNegPos(ang_rad, M_PI);
}
//! 角度 [rad] を [deg] に変換
inline double rad2deg(double ang_rad)
{
    return ang_rad / M_PI * 180.0;
}
//! 角度 [deg] を [rad] に変換
inline double deg2rad(double ang_deg)
{
    return ang_deg / 180.0 * M_PI;
}

// geometry.hpp
//! Differential Drive モーション
struct Velocity2d {
    double v;  // [m/s]
    double w;  // [rad/s]

    Velocity2d(double in_v = 0.0, double in_w = 0.0) : v(in_v), w(in_w) {}

    bool operator==(const Velocity2d &rhs) const { return (this->v == rhs.v && this->w == rhs.w); }
    bool operator!=(const Velocity2d &rhs) const { return !(*this == rhs); }
};
//! 2D 位置（直交座標点と方向）
struct Pose2d {
    double x;   // [m]
    double y;   // [m]
    double th;  // [rad]

    Pose2d(double in_x = 0.0, double in_y = 0.0, double in_th = 0.0) : x(in_x), y(in_y), th(in_th) {}

    bool operator==(const Pose2d &rhs) const { return (this->x == rhs.x && this->y == rhs.y && this->th == rhs.th); }
    bool operator!=(const Pose2d &rhs) const { return !(*this == rhs); }
};

// odometer.hpp
class SimpleOdometer {
public:
    SimpleOdometer() = default;

    ~SimpleOdometer() = default;

    // Compute 1 step odometry from encoders
  void addStepEnc(int enc_l, int enc_r, double step_time)
    {
        assert(timestamp > 0.0);

	//wrapToZeroMax(double val, double max)
	std::cerr << enc_l << " " << wrapToZeroMax(double(enc_l), 255.0) << " " ;
	std::cerr << enc_r << "\n";
	/*
        odom_.th += w * step_time;
        odom_.th = wrapToNegPosPi(odom_.th);
        odom_.x += v * cos(odom_.th) * step_time;
        odom_.y += v * sin(odom_.th) * step_time;

        travel_dist_ += std::abs(v) * step_time;
        travel_ang_ += std::abs(w) * step_time;
	*/
    }


    void addStepVel(double v, double w, double step_time)
    {
        assert(timestamp > 0.0);

        odom_.th += w * step_time;
        odom_.th = wrapToNegPosPi(odom_.th);
        odom_.x += v * cos(odom_.th) * step_time;
        odom_.y += v * sin(odom_.th) * step_time;

        travel_dist_ += std::abs(v) * step_time;
        travel_ang_ += std::abs(w) * step_time;
    }


    Pose2d odomPose() const
    {
        return odom_;
    }

    double travelDist() const
    {
        return travel_dist_;
    }

    double travelAng() const
    {
        return travel_ang_;
    }

private:
    Pose2d odom_ = {0.0, 0.0, 0.0};

    double travel_dist_ = 0.0;  // 累積走行距離 [m]
    double travel_ang_ = 0.0;   // 累積回転角度 [rad]
};

} // namespace rona

#endif // RONA_UTIL_HPP
