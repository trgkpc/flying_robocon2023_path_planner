import torch
import torch.nn as nn
import numpy as np

debug = True

def optimize(f, init, beta, lr, alpha, gamma, outer_n=9000, inner_n=20, threshold=1e-4, verbose=False):
    x = init.clone()
    w = torch.zeros_like(x)
    fx = f(x).item()
    for i in range(outer_n):
        x.requires_grad = True
        fx_ = f(x)
        fx = fx_.item()
        fx_.backward()
        dfdx = x.grad.clone()
        x = x.detach()

        w = beta * w + (1-beta) * dfdx
        d = -lr * w.clone()
        armijo = alpha * torch.dot(d, dfdx)
        for j in range(inner_n):
            dfx = f(x+d) - fx
            if dfx < armijo:
                break
            d *= gamma
            armijo *= gamma
        else:
            if verbose:
                print("[[WARNING]] max loop! (linear search)")
            break
        x = (x+d).clone()

        if verbose:
            print(i, fx, x)

        if fx < threshold:
            break
    else:
        if verbose:
            print("[[WARNING]] max loop! (gradient descent)")
    if verbose:
        print("final:", x)
    return x

def S(t, a, b, c):
    return torch.sqrt(a*(t**2) + b*t + c)

def calc_integration(t, a, b, c):
    st = S(t, a, b, c)
    s0 = S(0, a, b, c)
    st0 = st - s0
    tst = t * st

    sqa = torch.sqrt(a)
    re_sqa = 1 / sqa

    I = lambda x,s:re_sqa*torch.log(torch.abs(2*a*x+b+2*sqa*s))
    it = I(t,st)
    i0 = I(0,s0)
    it0 = it - i0
    tit = t * it

    q1 = it0
    p1 = (st0 - b * it0 / 2) / a
    q2 = -st0 / a + (b/(2*a))*it0 + tit - t * i0
    p2 = (3*b)/(4*(a*a)) * st0 + (4*a*c-3*(b*b))/(8*(a*a))*it0 + \
        1/(2*a) * tst - b/(2*a) * tit - (1/a) * t * s0 + \
        b / (2*a) * t * i0
    return [q1, p1, q2, p2]

def acc_lim_route_calc(tf, x0, v0, px, pv0, sqrt_acc=1):
    a = torch.dot(px, px)
    b = -2*torch.dot(px, pv0)
    c = torch.dot(pv0, pv0)
    
    if (b*b).item() == (4*a*c).item():
        acc = pv0 * (sqrt_acc * sqrt_acc / torch.sqrt(c))
        if b.item() >= 0 or tf <= (-b.item()/(2*a.item())):
            # 一定加速度
            vf = acc * tf + v0
            xf = 0.5 * acc * (tf*tf) + v0 * tf + x0
        else:
            # 前半
            t = -b/(2*a)
            v0 = acc * t + v0
            x0 = 0.5 * acc * (t*t) + v0 * t + x0
            # 後半
            tf = tf - t
            acc = -acc
            vf = acc * tf + v0
            xf = 0.5 * acc * (tf*tf) + v0 * tf + x0
    else:
        q1,p1,q2,p2 = calc_integration(tf*sqrt_acc, a, b, c)
        xf = (q2 * pv0 - p2 * px) + v0 * tf + x0
        vf = sqrt_acc * (q1 * pv0 - p1 * px) + v0
    
    return [xf, vf]

def solve_impl(x0, v0, xf, vf, init, sqrt_acc, verbose=False):
    Lp = nn.MSELoss()
    relu = nn.ReLU()
    def f(x):
        xf_, vf_ = acc_lim_route_calc(x[0], x0, v0, x[1:3], x[3:5], sqrt_acc)
        return relu(-x[0]) + Lp(xf_, xf) + Lp(vf_, vf)

    beta = 0.95
    lr = 0.05
    alpha = 0.7
    gamma = 0.5
    
    return optimize(f, init, beta, lr, alpha, gamma, verbose=verbose)

def solve(x0, v0, xf, vf, init, sqrt_acc, verbose=False):
    x0 = np.array(x0, dtype=float)
    v0 = np.array(v0, dtype=float)
    xf = np.array(xf, dtype=float)
    vf = np.array(vf, dtype=float)
    init = np.array(init, dtype=float)
    return solve_impl(
        torch.from_numpy(x0), 
        torch.from_numpy(v0),
        torch.from_numpy(xf), 
        torch.from_numpy(vf),
        torch.from_numpy(init),
        sqrt_acc,
        verbose
    )
    
def acc_lim_route_calc(tf, x0, v0, px, pv0, sqrt_acc=1):
    a = torch.dot(px, px)
    b = -2*torch.dot(px, pv0)
    c = torch.dot(pv0, pv0)
    
    if (b*b).item() == (4*a*c).item():
        acc = pv0 * (sqrt_acc * sqrt_acc / torch.sqrt(c))
        if b.item() >= 0 or tf <= (-b.item()/(2*a.item())):
            # 一定加速度
            vf = acc * tf + v0
            xf = 0.5 * acc * (tf*tf) + v0 * tf + x0
        else:
            # 前半
            t = -b/(2*a)
            v0 = acc * t + v0
            x0 = 0.5 * acc * (t*t) + v0 * t + x0
            # 後半
            tf = tf - t
            acc = -acc
            vf = acc * tf + v0
            xf = 0.5 * acc * (tf*tf) + v0 * tf + x0
    else:
        q1,p1,q2,p2 = calc_integration(tf*sqrt_acc, a, b, c)
        xf = (q2 * pv0 - p2 * px) + v0 * tf + x0
        vf = sqrt_acc * (q1 * pv0 - p1 * px) + v0
    
    return [xf, vf]

if __name__ == '__main__':
    x0 = torch.tensor([   0,  0.])
    v0 = torch.tensor([   0,  0.])
    xf = torch.tensor([  1.,  0.])
    vf = torch.tensor([   0,  0.])
    init = [ 2, 0.1, 1., 2., 1.]
    params = torch.tensor(init)
    
    import time
    t0 = time.time()
    x = solve_impl(x0, v0, xf, vf, params, 1.0, True)
    print(list(x))
    print(time.time()-t0)

