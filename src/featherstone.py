#!/usr/bin/python
import numpy
from matrixfunc import *
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

nlink = 10 # Number of links
mass    = 0.5 * numpy.ones(nlink+1)
llen    = 1.0 * numpy.ones(nlink+1) # length of links
lgc     = llen/2.0  # center of mass
jtype   = numpy.zeros(nlink)  # types of joints
lmd     = numpy.array([i for i in range(-1, nlink-1)])
inertia = [numpy.mat(zeros((6, 6))) for i in range(nlink+1)]
for i in range(nlink+1):
    inertia[i][1,1] =(mass[i] * llen[i]**2)/12.0
    inertia[i][2,2] =(mass[i] * llen[i]**2)/12.0
    inertia[i][3:6,3:6] = mass[i] * E3
    htmp = hat(numpy.mat([[lgc[i]], [0.0], [0.0]]))
    inertia[i][0:3,0:3] = inertia[i][0:3,0:3] + htmp*htmp.T
    inertia[i][0:3,3:6] = mass[i] * htmp
    inertia[i][3:6,0:3] = mass[i] * htmp.T
GRAVITY = 9.8

delta_t  = 0.002
tstep    = 5000
eps      = 1e-5

# Joint Types
#0:Revolute Joint
#1:Prismatic Joint
#2:6dof Free Joint
def jcalc(jtype, q, qdot, p):
    if jtype == 0: #Revolute Joint
        S = numpy.mat([[0.0],[0.0],[1.0],[0.0],[0.0],[0.0]])
        vj = S * qdot
        cj = numpy.mat(numpy.zeros((6, 1)))
        Xj = spRot(makeRot(2, q).T)
    return S, Xj, vj, cj

def ArticulatedBodyAlgorithm(nlink, q, qdot,
                             tau, fx, p,
                             inertia, lmd,
                             jtype, Xt,
                             a0 = numpy.mat([[0.0], [0.0], [0.0], [0.0], [GRAVITY], [0.0]])):
    """
    Articulated-Body Algorithm
    """
    S = [numpy.mat(numpy.zeros((6, 1))) for _ in range(nlink)]
    IAi = [numpy.mat(numpy.zeros((6, 6))) for _ in range(nlink)]
    pAi = [numpy.mat(numpy.zeros((6, 1))) for _ in range(nlink)]
    ci  = [numpy.mat(numpy.zeros((6, 1))) for _ in range(nlink)]
    i_X_li = [numpy.mat(E6) for _ in range(nlink)]
    i_X_o  = [numpy.mat(E6) for _ in range(nlink)]
    U = [numpy.mat(numpy.zeros((6, 1))) for _ in range(nlink)]
    D = [numpy.mat(numpy.zeros((1, 1))) for _ in range(nlink)]
    u = [numpy.mat(numpy.zeros((6, 1))) for _ in range(nlink)]
    IAi = [numpy.mat(numpy.zeros((6, 6))) for _ in range(nlink)]
    vi   = [numpy.mat(numpy.zeros((6, 1))) for _ in range(nlink)]
    a   = [numpy.mat(numpy.zeros((6, 1))) for _ in range(nlink)]
    # Phase1
    for i in range(nlink):
        S[i], Xj, vj, cj = jcalc(jtype[i], q[i,0], qdot[i,0], p[i])
        if lmd[i] != -1:
            i_X_li[i] = Xj * Xt[i]
            i_X_o[i] = i_X_li[i] * i_X_o[lmd[i]]
            vi[i] = i_X_li[i] * vi[lmd[i]]+vj
        else:
            i_X_li[i] = Xj * Xt[i]
            i_X_o[i] = i_X_li[i]
            vi[i] = vj
        ci[i] = cj + spCross(vi[i], vj)
        IAi[i] = inertia[i]
        pAi[i] = spDualCross(vi[i], inertia[i] * vi[i]) - spDual(i_X_o[i]) * fx[i]
    # Phase2
    for i in range(nlink-1, -1, -1):
        U[i] = IAi[i] * S[i]
        D[i] = S[i].T * U[i]
        u[i] = S[i].T * (tau[i] - pAi[i])
        if lmd[i] != -1:
            Ia = IAi[i] - U[i] * D[i].I * U[i].T
            pa = pAi[i] + Ia * ci[i] + U[i] * D[i].I * u[i]
            IAi[lmd[i]] = IAi[lmd[i]] + spDualInv(i_X_li[i]) * Ia * i_X_li[i]
            pAi[lmd[i]] = pAi[lmd[i]] + spDualInv(i_X_li[i]) * pa
    # Phase3
    for i in range(nlink):
        if lmd[i] != -1:
            adsh = i_X_li[i] * a[lmd[i]] + ci[i]
        else:
            adsh = i_X_li[i] * a0 + ci[i]
        q2dot[i] = D[i].I * (u[i] - U[i].T * adsh)
        a[i] = adsh + S[i] * q2dot[i]

    return q2dot, i_X_o

def fwdkinematics(pl, X):
    """
    forward kinematics
    """
    n = len(X)
    pos = [numpy.mat(numpy.zeros((3, 1))) for i in range(n)]
    pos[0] = X[0][0:3, 0:3].T * pl[1];
    for i in range(1, n):
        pos[i] = pos[i-1] + X[i][0:3, 0:3].T * pl[i+1];
    return pos

if __name__ == "__main__":
    q = numpy.mat(numpy.zeros((nlink, 1)))
    qdot = numpy.mat(numpy.zeros((nlink, 1)))
    q2dot = numpy.mat(numpy.zeros((nlink, 1)))

    fx = [numpy.mat(numpy.zeros((6, 1))) for i in range(nlink)]
    tau = [numpy.mat(numpy.zeros((6, 1))) for i in range(nlink)]
    p = [numpy.mat(numpy.zeros((3, 1)))] + [numpy.mat([[llen[i]],[0.0],[0.0]]) for i in range(1, nlink+1)]
    Xt = [numpy.mat(E6)] + [spXlt(p[i]) for i in range(1, nlink+1)]
    i_X_o  = [numpy.mat(E6) for i in range(nlink)]
    pos = fwdkinematics(p, i_X_o)

    def calc_state(q, qdot):
        q2dot, i_X_o = ArticulatedBodyAlgorithm(nlink, q, qdot, tau, fx, p, inertia, lmd, jtype, Xt);
        qdot = qdot + delta_t * q2dot
        q = q + delta_t * qdot
        return q, qdot, i_X_o

    # Draw
    fig = plt.figure()

    ax = fig.add_subplot(111, aspect='equal')
    line, = ax.plot([0]+[pos[i][0, 0] for i in range(nlink)], [0]+[pos[i][1, 0] for i in range(nlink)],'-o')
    maxval = 1.0*nlink + 0.5
    plt.xlim(-maxval, maxval)
    plt.ylim(-maxval, maxval)
    def update_draw(event):
        global q, qdot
        q, qdot, i_X_o = calc_state(q, qdot)
        pos = fwdkinematics(p, i_X_o)
        line.set_xdata([0]+[pos[i][0, 0] for i in range(nlink)])
        line.set_ydata([0]+[pos[i][1, 0] for i in range(nlink)])
        fig.canvas.draw()                 # redraw the canvas
        return line,

    ani = animation.FuncAnimation(fig, update_draw, arange(tstep), interval=5)
    plt.show()
