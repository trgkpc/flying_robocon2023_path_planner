import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
import time
import os

print("initilizing...")

from route.basic_route import CircularRoute, ConstVelRoute, ConstAccRoute, GoalPoint
from route.acc_lim_route import AccLimRoute
from tools.visualize import visualize_route, visualize_xy, visualize_vel
from tools.solver import solve
from util import e as unit_vec

# 我々が設計するパラメタ
linear_velocity = 3.0
linear_acc_lim = 1.0
circular_acc_lim = 2**0.5
poll_margin = 2.5

# 内部的に保持するパラメタ
a = [4.102178769569075, 2.6982251416174923, -0.8251836461275666, 2.1349253356003157e-05, -4.939585580773475]
b = [4.591587976825632, 3.1576045197942233, 1.1132819452243086, 7.262008599711447, 2.558352481341658]
c = [4.880759586195317, 1.451204526070504, -0.20748774736900066, 3.4079966444681182, 0.21879636272755154]
hachinoji_start_x = 20.06507528974203

route_inds = [3, 5, 7]
acc_lims = [linear_acc_lim, linear_acc_lim, linear_acc_lim]

def normalized(x):
    return x/LA.norm(x)

def make_routes(a, b, c, hachinoji_start_x, normalize):
    LINE_VEL = linear_velocity
    LINE_ACC_SCALE = linear_acc_lim
    LINE_ACC = linear_acc_lim**2
    CIRC_ACC = circular_acc_lim**2
    POLL_MARGIN = poll_margin

    INIT_X = -1.75
    ACC_END_X = INIT_X + (LINE_VEL**2) / (2*LINE_ACC)
    POLL_X = 25
    POLL_Y = 6
    MAIN_MISSION_X = 10.5
    GOAL_X = POLL_X + 3

    CIRC_VEL = (POLL_MARGIN*CIRC_ACC)**0.5

    tfs,init_costates = [],[]
    for params in [a,b,c]:
        tfs.append(params[0])
        init_costates.append([[params[1],params[2]],[params[3],params[4]]])

    def get_opt_route(i, init_state):
        tf = tfs[i]
        init_costate = init_costates[i]
        acc_lim = acc_lims[i]

        x,v = init_state
        px,pv = np.array(init_costate)
        ex = normalized(px)
        ev = normalized(pv)
        a = ex * acc_lim * acc_lim

        if (not normalize) or abs(ex@ev) < (1-1e-4):
            return [AccLimRoute(tf, init_state, init_costate, acc_lim)]
        else:
            X = px@ex
            V = pv@ex
            T = -1 if X<=0 else V/X
            print("normalize route",i, f"{ex@ev=}", f"{X=}", f"{V=}", f"{T=}", end=" ")
            if T < 0 or T >= tf:
                print("const acc.")
                return [ConstAccRoute(x, v, a, tf)]
            else:
                print("acc & dec.", T, tf)
                route0 = ConstAccRoute(x, v, a, T)
                x,v = route0(1.0)
                return [route0, ConstAccRoute(x, v, -a, tf-T)]

    one_dim_t0 = b[0] - b[2]
    one_dim_t1 = b[2]
    one_dime_acc = LINE_ACC * unit_vec(b[1])
    one_dim_v_ts = np.array([0, CIRC_VEL]) + one_dim_t0 * one_dime_acc

    routes = [
        # 加速
        ConstAccRoute([INIT_X, 0.], [0., 0.], [LINE_ACC, 0.], LINE_VEL / LINE_ACC),
        # 物資投下
        ConstVelRoute([ACC_END_X, 0.], [LINE_VEL, 0.], (MAIN_MISSION_X-ACC_END_X) / LINE_VEL),
        # ８の字侵入
        ConstVelRoute([MAIN_MISSION_X, 0.], [LINE_VEL, 0.], (hachinoji_start_x-MAIN_MISSION_X) / LINE_VEL),
        ] + get_opt_route(0, [[hachinoji_start_x, 0.], [LINE_VEL, 0]]) + [ 
        # ８の字円弧
        CircularRoute([POLL_X,-POLL_Y], POLL_MARGIN, 0.0,-np.pi, CIRC_VEL),
        # ８の字中央
        ] + get_opt_route(1, [[POLL_X-POLL_MARGIN,-POLL_Y], [0, CIRC_VEL]]) + [ 
        # ８の字円弧
        CircularRoute([POLL_X, POLL_Y], POLL_MARGIN, 0.0, np.pi, CIRC_VEL),
        # ８の字終了
        ] + get_opt_route(2, [[POLL_X-POLL_MARGIN, POLL_Y], [0,-CIRC_VEL]]) + [ 
        # 停止
        GoalPoint(GOAL_X, 0)
    ]
    
    return routes

def get_routes(normalize=True):
    return make_routes(a, b, c, hachinoji_start_x, normalize)

def optimize(i):
    r = route_inds[i]

    routes = get_routes(normalize=False)
    x0,v0 = routes[r](0.)
    xf,vf = routes[r+1](0.)
    init = [a,b,c][i]
    verbose = False

    print("optimizing...")
    t0 = time.time()
    ret = solve(x0, v0, xf, vf, init, acc_lims[i], verbose).tolist()
    print("route", i, "calculated. time:", time.time()-t0, "sec")
    return ret

def fix_hachinoji_start_pos():
    global a
    i = 0
    r = route_inds[i]
    dx = 0.1
    threshold = 1e-5
    lr = 0.1
    verbose = True

    routes = get_routes(normalize=False)
    x0,v0 = routes[r](0.)
    xf,vf = routes[r+1](0.)

    def f(x):
        global a
        init = a
        X0 = [x, x0[1]]
        ret = solve(X0, v0, xf, vf, init, acc_lims[i]).tolist()
        a = ret
        return ret[3] / abs(ret[4])

    x = hachinoji_start_x
    fx = f(x)
    for j in range(1000):
        if abs(fx) < threshold:
            break
        fxdx = f(x+dx)
        dfdx = (fxdx-fx) / dx

        d = -fx / dfdx
        ratio = 1.0
        dmax = 0.5
        if abs(d) > dmax:
            ratio = dmax / abs(d)
        fx_ = f(x+ratio*d)
        for k in range(20):
            if fx_ * fx < 0:
                ratio *= abs(fx) / (abs(fx)+abs(fx_))
                print("     ->", ratio)
                fx_ = f(x+d*ratio)
                continue
            if abs(fx_) < 0.9 * abs(fx):
                break
            if verbose:
                print("  ", k, fx_, ratio)
            ratio *= 0.8
            fx_ = f(x+d*ratio)
        if verbose:
            print(j, x, fx, fx_)

        if abs(fx_) > abs(fx):
            print("[error] hachinoji optimization error")
            break
            
        fx = fx_

        x += d*ratio
    return x

def calc():
    global a,b,c,hachinoji_start_x
    print("initial optimization....")
    t0 = time.time()
    a = optimize(0)
    b = optimize(1)
    c = optimize(2)
    print("initial optimization end. time:", time.time()-t0)
    
    print("total optimization....")
    t0 = time.time()
    hachinoji_start_x = fix_hachinoji_start_pos()
    print("total optimization end. time:", time.time()-t0)

if __name__ == '__main__':
    calc()
    print("route calculation end.")
    print()
    print("以下のパラメタを main.py の # 内部的に保持するパラメタ 以下の行に保存してください)")
    print("=================")
    print("a =", a)
    print("b =", b)
    print("c =", c)
    print("hachinoji_start_x =", hachinoji_start_x)
    print("=================")
    print()

    routes = get_routes()

    os.makedirs("output", exist_ok=True)
    visualize_route(routes, "output/image.png")
    visualize_xy(routes, "output/xy.png")
    visualize_vel(routes, "output/vel.png")

    print("以下のパラメタを cpp/test/route.* に保存してください")
    print("===== route.cpp =====")
    for i,route in enumerate(routes):
        r = str(route)
        t = r.split("(")[0]
        print(f"    route{i} = {r};")
    print("===== route.h =====")
    for i,route in enumerate(routes):
        r = str(route)
        t = r.split("(")[0]
        print(f"    {t} route{i};")
    print("=================")

    
    t_total = sum([route.tf for route in routes])
    print(f"{t_total=}")
