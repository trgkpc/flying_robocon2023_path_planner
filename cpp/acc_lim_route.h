#pragma once

#include "basic_route.h"
#include "calc_acc_lim_route.h"

namespace Route
{
using linear::Vec2f;

struct AccLimRoute : AbstRoute
{
    AccLimRoute(){}
    AccLimRoute(float tf_, 
                const Vec2f& x0_, const Vec2f& v0_, 
                const Vec2f& px_, const Vec2f& pv0_, float sqrt_acc_)
        : x0(x0_), v0(v0_), px(px_), pv0(pv0_), sqrt_acc(sqrt_acc_)
    {
        tf = tf_;
    }

    RouteDest operator()(float t) override
    {
        return AccLimRouteCalc::calc_acc_lim_route(t, x0, v0, px, pv0, sqrt_acc);
    }
private:
    Vec2f x0;
    Vec2f v0;
    Vec2f px;
    Vec2f pv0;
    float sqrt_acc;
};

} // namespace Route
