#pragma once
#include "linear_algebra/Vec2.hpp"

namespace Route
{
using linear::Vec2f;

struct RouteDest
{
    RouteDest(const Vec2f& p)
        : pos(p), vel(Vec2f(0.0f, 0.0f)), acc(Vec2f(0.0f, 0.0f)), index(-1)
    {
    }

    RouteDest(const Vec2f& p, const Vec2f& v)
        : pos(p), vel(v), acc(Vec2f(0.0f, 0.0f)), index(-1)
    {
    }

    RouteDest( const Vec2f& p, const Vec2f& v,const Vec2f& a)
        : pos(p), vel(v), acc(a), index(-1)
    {
    }

    int index;
    Vec2f pos;
    Vec2f vel;
    Vec2f acc;
};
}
