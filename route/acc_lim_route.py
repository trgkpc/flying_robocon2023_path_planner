import numpy as np

from calc_acc_lim_route import acc_lim_route as acc_lim_route_calc
from util import str_vec2

class VelScaledAccLimRoute:
    def __init__(self, params, scaling_param=1.0):
        self.tf = params[0] / scaling_param
        self.init_state = params[1]
        self.init_state[1][0] *= scaling_param
        self.init_state[1][1] *= scaling_param
        self.end_state = params[2]
        self.end_state[1][0] *= scaling_param
        self.end_state[1][1] *= scaling_param
        self.init_costate = params[3]
        self.scaling_param = scaling_param

    def __call__(self, ratio):
        state = acc_lim_route_calc(
            self.tf * ratio,
            self.init_costate[0],
            self.init_costate[1],
            self.init_state[0],
            self.init_state[1],
            self.scaling_param
        )

        return state

class AccLimRoute:
    def __init__(self, tf, init_state, init_costate, scaling_param):
        self.tf = tf
        self.init_state = init_state
        self.init_costate = init_costate
        self.scaling_param = scaling_param

    def __call__(self, ratio):
        state = acc_lim_route_calc(
            self.tf * ratio,
            self.init_costate[0],
            self.init_costate[1],
            self.init_state[0],
            self.init_state[1],
            self.scaling_param
        )

        return state

    def __str__(self):
        a = f"{str_vec2(self.init_state[0])}, {str_vec2(self.init_state[1])}"
        b = f"{str_vec2(self.init_costate[0])}, {str_vec2(self.init_costate[1])}"
        return f"AccLimRoute({self.tf}f, {a}, {b}, {self.scaling_param}f)"

