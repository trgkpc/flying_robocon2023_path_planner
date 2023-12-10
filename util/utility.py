import numpy as np
    
def e(theta):
    return np.array([np.cos(theta), np.sin(theta)])

def str_vec2(v):
    return f"linear::Vec2f({float(v[0])}f, {float(v[1])}f)"

