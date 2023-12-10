import numpy as np

from util import e, str_vec2

class CircularRoute:
    def __init__(self, center, radius, theta0, thetaf, v):
        self.r0 = np.array(center)
        self.R = radius
        self.theta0 = theta0
        self.dtheta = thetaf - theta0
        self.v = v
        self.reverse = (self.dtheta < 0)
        self.tf = abs(radius * self.dtheta / v)

    def __call__(self, ratio):
        theta = self.theta0 + ratio * self.dtheta
        r = self.r0 + self.R * e(theta)
        if self.reverse:
            v = self.v * e(theta - 0.5*np.pi)
        else:
            v = self.v * e(theta + 0.5*np.pi)

        return np.array([r, v])
    
    def __str__(self):
        return f"CircularRoute({str_vec2(self.r0)}, {self.R}f, {self.theta0}f, {self.dtheta}f, {self.v}f)"

class ConstVelRoute:
    def __init__(self, r0, v, t):
        self.r0 = np.array(r0)
        self.v = np.array(v)
        self.tf = t
    
    def __call__(self, ratio):
        t = ratio * self.tf
        r = self.r0 + t * self.v
        return np.array([r, self.v])

    def __str__(self):
        return f"ConstVelRoute({str_vec2(self.r0)}, {str_vec2(self.v)}, {self.tf}f)"

class ConstAccRoute:
    def __init__(self, r0, v0, a, t):    
        self.r0 = np.array(r0)
        self.v0 = np.array(v0)
        self.a = np.array(a)
        self.tf = t
    
    def __call__(self, ratio):
        t = ratio * self.tf
        v = self.v0 + t * self.a
        r = self.r0 + t * self.v0 + 0.5*(t**2) * self.a
        return np.array([r, v])

    def __str__(self):
        return f"ConstAccRoute({str_vec2(self.r0)}, {str_vec2(self.v0)}, {str_vec2(self.a)}, {self.tf}f)"

class GoalPoint:
    def __init__(self, x0, y0):
        self.r = np.array([x0, y0])
        self.tf = 0.

    def __call__(self, ratio):
        return np.array([self.r, [0,0]])

    def __str__(self):
        return f"GoalPoint({str_vec2(self.r)})"

if __name__ == '__main__':
    route = ConstAccRoute([-1.75, 0.], [0., 0.], [2, 0.], 1)
    print(route(1.0).tolist())

