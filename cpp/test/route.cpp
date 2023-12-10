#include "route.h"

namespace MyRoute
{
HachinojiRoute hachinoji_route;

void initializeRoute()
{
  hachinoji_route = HachinojiRoute();
}

HachinojiRoute::HachinojiRoute()
{
    // 以下に貼り付け
    route0 = ConstAccRoute(linear::Vec2f(-1.75f, 0.0f), linear::Vec2f(0.0f, 0.0f), linear::Vec2f(1.0f, 0.0f), 3.0f);
    route1 = ConstVelRoute(linear::Vec2f(2.75f, 0.0f), linear::Vec2f(3.0f, 0.0f), 2.5833333333333335f);
    route2 = ConstVelRoute(linear::Vec2f(10.5f, 0.0f), linear::Vec2f(3.0f, 0.0f), 3.1883584299140098f);
    route3 = AccLimRoute(4.102179608840494f, linear::Vec2f(20.06507528974203f, 0.0f), linear::Vec2f(3.0f, 0.0f), linear::Vec2f(2.6982200725401144f, -0.8252056566562717f), linear::Vec2f(4.2695514635985225e-05f, -4.9395846726175945f), 1.0f);
    route4 = CircularRoute(linear::Vec2f(25.0f, -6.0f), 2.5f, 0.0f, -3.141592653589793f, 2.23606797749979f);
    route5 = ConstAccRoute(linear::Vec2f(22.5f, -6.0f), linear::Vec2f(0.0f, 2.23606797749979f), linear::Vec2f(0.9430999503908475f, 0.3325093736615272f), 2.299646156502995f);
    route6 = ConstAccRoute(linear::Vec2f(24.993731895320025f, 0.021381834844515724f), linear::Vec2f(2.1687961761144776f, 3.000721880641739f), linear::Vec2f(-0.9430999503908475f, -0.3325093736615272f), 2.2919425996236f);
    route7 = CircularRoute(linear::Vec2f(25.0f, 6.0f), 2.5f, 0.0f, 3.141592653589793f, 2.23606797749979f);
    route8 = AccLimRoute(4.8807598757476525f, linear::Vec2f(22.5f, 6.0f), linear::Vec2f(0.0f, -2.23606797749979f), linear::Vec2f(1.4511808199176068f, -0.2074755966680225f), linear::Vec2f(3.4080067672991667f, 0.2188074462011491f), 1.0f);
    route9 = GoalPoint(linear::Vec2f(28.0f, 0.0f));
}

void HachinojiRoute::initialize(float t)
{
    t0 = t;
    is_end = false;
}


#define call_route(i)           \
    if(t < route##i.getTime()){ \
        auto ret = route##i(t); \
        ret.index = i;          \
        return ret;             \
    }                           \
    t -= route##i.getTime();

#define call_end_route(i)               \
    is_end = (t >= route##i.getTime()); \
    auto ret = route##i(t);             \
    ret.index = i;                      \
    return ret;

RouteDest HachinojiRoute::operator()(float t)
{
    t -= t0;
    call_route(0);
    call_route(1);
    call_route(2);
    call_route(3);
    call_route(4);
    call_route(5);
    call_route(6);
    call_route(7);
    call_route(8);
    call_end_route(9);
}

float HachinojiRoute::getTime()
{
    float t = 0;
    t += route0.getTime();
    t += route1.getTime();
    t += route2.getTime();
    t += route3.getTime();
    t += route4.getTime();
    t += route5.getTime();
    t += route6.getTime();
    t += route7.getTime();
    t += route8.getTime();
    t += route9.getTime();
    return t;
}
} // namespace MyRoute

