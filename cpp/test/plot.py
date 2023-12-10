import numpy as np
import matplotlib.pyplot as plt

def main(fname):
    log = []
    for l in open(fname, "r").readlines():
        if "nan" in l:
            continue
        log.append(list(map(float, l.split())))

    log = np.array(log).T

    T = log[0]
    I = log[1]
    R = log[2:4]
    V = log[4:6]
    A = log[6:8]
    n = int(np.max(I)+1)

    def integral(x0, dxdt):
        x = np.array(x0)
        X = [np.array(x)]
        for i in range(len(T)-1):
            x += dxdt[:,i] * (T[i+1]-T[i])
            X.append(np.array(x))
        X = np.array(X)
        print(X.shape)
        return X.T

    def route(XY):
        x,y = XY
        for i in range(n):
            ind = I==i
            plt.plot(x[ind], y[ind])
        plt.show()
        
    def txy(XY):
        x,y = XY
        for i in range(n):
            ind = I==i
            plt.plot(T[ind], x[ind])
            plt.plot(T[ind], y[ind])
        plt.show()

    R_ = integral(R[:,0], V)
    V_ = integral(V[:,0], A)

    #route(R)
    #route(R_)
    txy(V)
    txy(V_)
    txy(A)

main("hachinoji_route.log")
