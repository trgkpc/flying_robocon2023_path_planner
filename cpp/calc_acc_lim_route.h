#pragma once

#include <array>
#include <cmath>

#include "linear_algebra/Vec2.hpp"
#include "basic_route.h"

namespace Route::AccLimRouteCalc
{
using linear::Vec2f;

inline std::array<float, 4> calc_integration_of(float t, float a, float b, float c)
{
    using std::sqrt, std::log, std::abs;
    float st = sqrt(a * (t * t) + b * t + c);
    float s0 = sqrt(c);
    float st0 = st - s0;
    float tst = t * st;

    float sqa = sqrt(a);
    float re_sqa = 1 / sqa;

    float it = re_sqa * log(abs(2 * a * t + b + 2 * sqa * st));
    float i0 = re_sqa * log(abs(b + 2 * sqa * s0));
    float it0 = it - i0;
    float tit = t * it;

    float q1 = it0;
    float p1 = (st0 - 0.5f * b * it0) / a;
    float q2 = -st0 / a + (b / (2 * a)) * it0 + tit - t * i0;
    float p2 = (3 * b) / (4 * (a * a)) * st0 + (4 * a * c - 3 * (b * b)) / (8 * (a * a)) * it0 + 1 / (2 * a) * tst - b / (2 * a) * tit - (1 / a) * t * s0 + b / (2 * a) * t * i0;
    return std::array<float, 4>{{q1, p1, q2, p2}};
}

inline RouteDest calc_acc_lim_route(float tf,
            const Vec2f& x0, const Vec2f& v0, 
            const Vec2f& px, const Vec2f& pv0,
            float sqrt_acc) 
{
    Vec2f xtf, vtf, acc;
    float a = px*px;
    float b = (-2)*(px*pv0);
    float c = pv0*pv0;

    if ((b * b) == (4.0 * a * c)) {
        Vec2f a0 = pv0 * (sqrt_acc * sqrt_acc / sqrt(c));
        if (b >= 0 || tf<=(-b/(2*a))) {
            // 一定加速度
            vtf = a0 * tf + v0;
            xtf = 0.5 * a0 * (tf * tf) + v0 * tf + x0;
            acc = a0;
        } else {
            // 前半
            float ta = -b / (2.0 * a);
            Vec2f vh = a0 * ta + v0;
            Vec2f xh = 0.5 * a0 * (ta * ta) + v0 * ta + x0;
            // 後半
            float tb = tf - ta;
            vtf = -a0 * tb + vh;
            xtf = -0.5 * a0 * (tb * tb) + vh * tb + xh;
            acc = -a0;
        }
    } else {
        std::array<float, 4> P = calc_integration_of(tf*sqrt_acc, a, b, c);
        float q1 = P[0];
        float p1 = P[1];
        float q2 = P[2];
        float p2 = P[3];

        xtf = (q2 * pv0 - p2 * px) + v0 * tf + x0;
        vtf = sqrt_acc * (q1 * pv0 - p1 * px) + v0;

        Vec2f pv = pv0 - tf * px;
        acc = (sqrt_acc*sqrt_acc) * pv.normalized();
    }

    return RouteDest(xtf, vtf, acc);
}

} // namespace Route::AccLimRouteCalc


