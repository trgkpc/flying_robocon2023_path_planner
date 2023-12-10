#pragma once
#include "datatype.h"
#include "bangbang.h"

namespace Route
{
using linear::Vec2f;

struct AbstRoute
{
    virtual RouteDest operator()(float t) = 0;
    float getTime() { return tf; }
protected:
    float tf;
};

struct CircularRoute : AbstRoute
{
    CircularRoute(){}
    CircularRoute(const Vec2f& r0_, float R_, float theta0_, float dtheta_, float V_)
        : r0(r0_), R(R_), theta0(theta0_), dtheta(dtheta_), V(V_)
    {
        A = V_ * V_ / R_;
        tf = std::abs(R_ * dtheta_ / V_);
        if(dtheta_ > 0.0f){
            dt_rot_theta = 0.5f * M_PI;
        }else{
            dt_rot_theta = -0.5f * M_PI;
        }
    }

    RouteDest operator()(float t) override
    {
        const float ratio = t / tf;
        const float theta = theta0 + ratio * dtheta;

        Vec2f r = r0 + R * linear::e(theta);
        Vec2f v = V * linear::e(theta + dt_rot_theta);
        Vec2f a = A * linear::e(theta + 2 * dt_rot_theta);
        return RouteDest(r, v, a);
    }
private:
    Vec2f r0;
    float R;
    float theta0;
    float dtheta;
    float V;
    float A;
    float dt_rot_theta;
};

struct ConstVelRoute : AbstRoute
{
    ConstVelRoute(){}
    ConstVelRoute(const Vec2f& r0_, const Vec2f& v_, float tf_)
        : r0(r0_), v(v_)
    {
        tf = tf_;
    }

    RouteDest operator()(float t) override
    {
        Vec2f r = r0 + v * t;
        return RouteDest(r, v);
    }

private:
    Vec2f r0;
    Vec2f v;
};

struct ConstAccRoute : AbstRoute
{
    ConstAccRoute(){}
    ConstAccRoute(const Vec2f& r0_, const Vec2f& v0_, const Vec2f& a_, float tf_)
        : r0(r0_), v0(v0_), a(a_)
    {
        tf = tf_;
    }

    RouteDest operator()(float t) override
    {
        Vec2f v = v0 + t * a;
        Vec2f r = r0 + t * v0 + (0.5f*t*t) * a;
        return RouteDest(r, v, a);
    }

private:
    Vec2f r0;
    Vec2f v0;
    Vec2f a;
};

struct GoalPoint : AbstRoute
{
    GoalPoint(){}
    GoalPoint(const Vec2f& r_) : r(r_)
    {
        tf = 0.0f;
    }

    RouteDest operator()(float t) override
    {
        return RouteDest(r);
    }

private:
    Vec2f r;
};

struct BangBangRoute : AbstRoute
{
    BangBangRoute(){}
    BangBangRoute(const Vec2f& r0_, const Vec2f& r_dest_, float v_lim, float a_lim)
    {
        r0 = r0_;
        r_dest = r_dest;
        Vec2f dr = r_dest - r0;
        
        e = dr.normalized();
        bangbang = BangBangPlanner(dr*e, v_lim, a_lim);
        tf = bangbang.getTime();
        is_end = false;
    }

    RouteDest operator()(float t) override
    {
        auto dest = bangbang(t);
        is_end = dest.is_end;
        return RouteDest(r0 + dest.x * e, dest.v * e, dest.a * e);
    }

    bool isEnd()
    {
        return is_end;
    }
private:
    Vec2f r0;
    Vec2f r_dest;
    Vec2f e;
    BangBangPlanner bangbang;
    bool is_end;
};
} // namespace Route
