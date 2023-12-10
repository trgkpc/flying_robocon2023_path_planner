#include <iostream>
#include <fstream>
#include <string>

#include "route.h"

template <class T>
void print(const std::string& fname, T& route)
{
    const int N = 30 * 1000;
    linear::Vec2f vec;
    float tf = route.getTime();
    std::ofstream os(fname);
    for(int i=0; i <= N; i++)
    {
        float t = float(i) / float(N) * tf;
        auto p_v_a = route(t);

        os << t << " " << p_v_a.index << " ";
        vec = p_v_a.pos;
        os << vec.x << " " << vec.y << " ";
        vec = p_v_a.vel;
        os << vec.x << " " << vec.y << " ";
        vec = p_v_a.acc;
        os << vec.x << " " << vec.y << std::endl;
    }
}

int main()
{
    MyRoute::initializeRoute();
    print("hachinoji_route.log", MyRoute::hachinoji_route);
}

