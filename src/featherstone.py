#!/usr/bin/python
import numpy as np
from matrixfunc import *

GRAVITY = 9.8
eps = 1e-5

# Joint Types
#0:Revolute Joint
#1:Prismatic Joint
#2:6dof Free Joint
def jcalc(jtype, q, qdot, p):
    if jtype == 0: #Revolute Joint
        S = np.array([[0.0],[0.0],[1.0],[0.0],[0.0],[0.0]])
        vj = S * qdot
        cj = np.zeros((6, 1))
        Xj = sp_rot(make_rot(2, q).T)
    else:
        raise NotImplementedError()
    return S, Xj, vj, cj

def articulated_body_algorithm(nlink, q, qdot,
                               tau, fx, p,
                               inertia, lmd,
                               jtype, Xt,
                               a0 = np.array([[0.0], [0.0], [0.0], [0.0], [GRAVITY], [0.0]])):
    """
    Articulated-Body Algorithm
    """
    S = np.zeros((nlink, 6, 1))
    IAi = np.zeros((nlink, 6, 6))
    pAi = np.zeros((nlink, 6, 1))
    ci  = np.zeros((nlink, 6, 1))
    i_X_li = np.tile(np.copy(E6), (nlink, 1, 1))
    i_X_o  = np.tile(np.copy(E6), (nlink, 1, 1))
    U = np.zeros((nlink, 6, 1))
    D = np.zeros((nlink, 1, 1))
    Dinv = np.zeros((nlink, 1, 1))
    u = np.zeros((nlink, 1, 1))
    IAi = np.zeros((nlink, 6, 6))
    vi = np.zeros((nlink, 6, 1))
    q2dot = np.zeros((nlink, 1))
    a = np.zeros((nlink, 6, 1))
    # Phase1
    for i in range(nlink):
        S[i], Xj, vj, cj = jcalc(jtype[i], q[i,0], qdot[i,0], p[i])
        if lmd[i] != -1:
            i_X_li[i] = np.matmul(Xj, Xt[i])
            i_X_o[i] = np.matmul(i_X_li[i], i_X_o[lmd[i]])
            vi[i] = np.matmul(i_X_li[i], vi[lmd[i]]) + vj
        else:
            i_X_li[i] = np.matmul(Xj, Xt[i])
            i_X_o[i] = i_X_li[i]
            vi[i] = vj
        ci[i] = cj + sp_cross(vi[i], vj)
        IAi[i] = inertia[i]
        pAi[i] = sp_dual_cross(vi[i], np.matmul(inertia[i], vi[i])) - np.matmul(sp_dual(i_X_o[i]), fx[i])
    # Phase2
    for i in range(nlink-1, -1, -1):
        U[i] = np.matmul(IAi[i], S[i])
        D[i] = np.matmul(S[i].T, U[i])
        Dinv[i] = np.linalg.inv(D[i])
        u[i] = tau[i] - np.matmul(S[i].T, pAi[i]) #np.matmul(S[i].T, (tau[i] - pAi[i]))
        if lmd[i] != -1:
            Ia = IAi[i] - np.matmul(np.matmul(U[i], Dinv[i]), U[i].T)
            pa = pAi[i] + np.matmul(Ia, ci[i]) + np.matmul(np.matmul(U[i], Dinv[i]), u[i])
            IAi[lmd[i]] = IAi[lmd[i]] + np.matmul(np.matmul(sp_dual_inv(i_X_li[i]), Ia), i_X_li[i])
            pAi[lmd[i]] = pAi[lmd[i]] + np.matmul(sp_dual_inv(i_X_li[i]), pa)
    # Phase3
    for i in range(nlink):
        if lmd[i] != -1:
            adsh = np.matmul(i_X_li[i], a[lmd[i]]) + ci[i]
        else:
            adsh = np.matmul(i_X_li[i], a0) + ci[i]
        q2dot[i] = np.matmul(Dinv[i], (u[i] - np.matmul(U[i].T, adsh)))
        a[i] = adsh + S[i] * q2dot[i]

    return q2dot, i_X_o

def fwdkinematics(pl, X):
    """
    forward kinematics
    """
    n = len(X)
    pos = np.zeros((n, 3, 1))
    pos[0] = np.matmul(X[0, 0:3, 0:3].T, pl[1])
    for i in range(1, n):
        pos[i] = pos[i-1] + np.matmul(X[i, 0:3, 0:3].T, pl[i+1])
    return pos

if __name__ == "__main__":
    import matplotlib
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    delta_t = 0.002

    nlink = 10 # Number of links
    mass = 0.5 * np.ones(nlink+1)
    llen = 1.0 * np.ones(nlink+1) # length of links
    lgc = llen/2.0  # center of mass
    jtype = np.zeros(nlink)  # types of joints
    lmd = [i for i in range(-1, nlink-1)]
    inertia = np.zeros((nlink+1, 6, 6))
    for i in range(nlink+1):
        inertia[i][1,1] =(mass[i] * llen[i]**2)/12.0
        inertia[i][2,2] =(mass[i] * llen[i]**2)/12.0
        inertia[i][3:6,3:6] = mass[i] * E3
        htmp = hat(np.array([[lgc[i]], [0.0], [0.0]]))
        inertia[i][0:3,0:3] = inertia[i][0:3,0:3] + np.matmul(htmp, htmp.T)
        inertia[i][0:3,3:6] = mass[i] * htmp
        inertia[i][3:6,0:3] = mass[i] * htmp.T

    q = np.zeros((nlink, 1))
    qdot = np.zeros((nlink, 1))
    q2dot = np.zeros((nlink, 1))

    fx = np.zeros((nlink, 6, 1))
    tau = np.zeros((nlink, 1))
    p = np.array([np.zeros((3, 1))] + [np.array([[llen[i]],[0.0],[0.0]]) for i in range(1, nlink+1)])
    Xt = np.array([np.copy(E6)] + [sp_xlt(p[i]) for i in range(1, nlink+1)])
    i_X_o  = np.tile(np.copy(E6), (nlink, 1, 1))
    pos = fwdkinematics(p, i_X_o)

    def calc_state(q, qdot):
        q2dot, i_X_o = articulated_body_algorithm(nlink, q, qdot, tau, fx, p, inertia, lmd, jtype, Xt);
        qdot = qdot + delta_t * q2dot
        q = q + delta_t * qdot
        return q, qdot, i_X_o

    # Draw
    fig = plt.figure()

    ax = fig.add_subplot(111, aspect='equal')
    line, = ax.plot([0]+[pos[i, 0, 0] for i in range(nlink)], [0]+[pos[i, 1, 0] for i in range(nlink)],'-o')
    maxval = 1.0*nlink + 0.5
    plt.xlim(-maxval, maxval)
    plt.ylim(-maxval, maxval)
    def update_draw(t):
        global q, qdot
        q, qdot, i_X_o = calc_state(q, qdot)
        pos = fwdkinematics(p, i_X_o)
        line.set_xdata([0]+[pos[i, 0, 0] for i in range(nlink)])
        line.set_ydata([0]+[pos[i, 1, 0] for i in range(nlink)])
        #if t % 10 == 0:
        #    plt.savefig("img_%04d.png" % t)
        return line,

    ani = animation.FuncAnimation(fig, update_draw, interval=5)
    plt.show()
