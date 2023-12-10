import numpy as np
import matplotlib.pyplot as plt
import multiprocessing

from util import draw_square_obj, draw_field
from main import get_routes

routes = get_routes()
tf = sum([route.tf for route in routes])

def get_pos_vel(t):
    for route in routes:
        if t < route.tf:
            return route(t/route.tf)
        t -= route.tf
    return routes[-1](1.)

def draw_machine(r):
    x,y = r
    plt.scatter(x, y, marker='x', color='blue', s=50, zorder=2)

dt = 0.05
n = int((tf + 1.0) / dt)
def f(i):
    fig = plt.figure(dpi=400)
    t = i * dt

    draw_field()

    r,v = get_pos_vel(t)
    draw_machine(r)

    print(t, r, v)
    fname = f"tmp/{str(i).zfill(4)}.png"
    plt.savefig(fname)
    plt.close()

with multiprocessing.Pool(processes=4) as pool:
    results = pool.map(f, list(range(n)))


