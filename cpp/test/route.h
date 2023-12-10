#pragma once
#include "../datatype.h"
#include "../basic_route.h"
#include "../acc_lim_route.h"

namespace MyRoute
{
using linear::Vec2f;
using Route::ConstAccRoute, Route::ConstVelRoute, Route::CircularRoute, Route::GoalPoint;
using Route::AccLimRoute;
using Route::RouteDest;

struct HachinojiRoute
{
    HachinojiRoute();
    void initialize(float t);
    RouteDest operator()(float t);
    float getTime();
    bool isEnd() { return is_end; }
private:

    ConstAccRoute route0;
    ConstVelRoute route1;
    ConstVelRoute route2;
    AccLimRoute route3;
    CircularRoute route4;
    ConstAccRoute route5;
    ConstAccRoute route6;
    CircularRoute route7;
    AccLimRoute route8;
    GoalPoint route9;
    
    float t0;
    bool is_end;
};

extern HachinojiRoute hachinoji_route;

void initializeRoute();
} // namespace MyRoute
