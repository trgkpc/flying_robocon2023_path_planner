import matplotlib.pyplot as plt
import numpy as np

from util import draw_field

def visualize_route(routes, fname=None):
    draw_field()
    for route in routes:
        r = [route(t)[0] for t in np.linspace(0., 1., 100)]
        x,y = np.array(r).T
        plt.plot(x, y)
    if fname is not None:
        plt.savefig(fname)
    plt.show()

def visualize_xy(routes, fname=None):
    tf = 0.0
    for route in routes:
        r = [route(t)[0] for t in np.linspace(0., 1., 100)]
        x,y = np.array(r).T
        t = tf + route.tf * np.linspace(0., 1., 100)
        plt.plot(t, x)
        plt.plot(t, y)

        tf += route.tf
    if fname is not None:
        plt.savefig(fname)
    plt.show()

def visualize_vel(routes, fname=None):
    tf = 0.0
    for route in routes:
        r = [route(t)[1] for t in np.linspace(0., 1., 100)]
        x,y = np.array(r).T
        t = tf + route.tf * np.linspace(0., 1., 100)
        plt.plot(t, x)
        plt.plot(t, y)

        v = (x**2+y**2)**0.5
        plt.plot(t, v)

        tf += route.tf
    if fname is not None:
        plt.savefig(fname)
    plt.show()

