#!/usr/bin/python
from numpy import *

X, Y, Z = range(3)

ex = matrix([[1.0], [0.0], [0.0]])
ey = matrix([[0.0], [1.0], [0.0]])
ez = matrix([[0.0], [0.0], [1.0]])
E3 = matrix(identity(3)) # 3x3 Identity
E6 = matrix(identity(6)) # 6x6 Identity

def hat(x):
    """
    outer product matrix
    """
    xhat = matrix(zeros((3, 3)))
    xhat[0, 1] = -x[2, 0]
    xhat[0, 2] = x[1, 0]
    xhat[1, 0] = x[2, 0]
    xhat[1, 2] = -x[0, 0]
    xhat[2, 0] = -x[1, 0]
    xhat[2, 1] = x[0, 0]
    return xhat

def sphat(x):
    """
    outer product matrix for spatial vector
    """
    xhat = matrix(zeros((6, 6)))
    xhat[0:3, 0:3] = hat(x[0:3, 0])
    xhat[3:6, 0:3] = hat(x[3:6, 0])
    xhat[3:6, 3:6] = hat(x[0:3, 0])
    return xhat

def mcross(v1, v2):
    """
    outer product for numpy matrix
    """
    ans = matrix(zeros((3, 1)))
    ans[0, 0] = v1[1, 0] * v2[2, 0] - v1[2, 0] * v2[1, 0]
    ans[1, 0] = v1[2, 0] * v2[0, 0] - v1[0, 0] * v2[2, 0]
    ans[2, 0] = v1[0, 0] * v2[1, 0] - v1[1, 0] * v2[0, 0]
    return ans

def spInv(X):
    """
    inverse spatial matrix
    """
    return matrix(bmat[[X[0:3, 0:3].T, X[0:3, 3:6].T],
                       [X[3:6, 0:3].T, X[3:6, 3:6].T]])

def spDual(X):
    return matrix(bmat([[X[0:3, 0:3], X[3:6, 0:3]],
                        [X[0:3, 3:6], X[3:6, 3:6]]]))

def spDualInv(X):
    """
    Conjugate inverse of spatial matrix
    """
    return matrix(bmat([[X[0:3, 0:3].T, X[3:6, 0:3].T],
                        [X[0:3, 3:6].T, X[3:6, 3:6].T]]))

def spCross(spv1, spv2):
    """
    Outer product for spatial vectors
    """
    return r_[mcross(spv1[0:3, 0], spv2[0:3, 0]),
              mcross(spv1[0:3, 0], spv2[3:6, 0]) + mcross(spv1[3:6, 0], spv2[0:3, 0])]

def spDualCross(spv1, spv2):
    return r_[mcross(spv1[0:3, 0], spv2[0:3, 0]) + mcross(spv1[0:3, 0], spv2[0:3, 0]),
              mcross(spv1[0:3, 0], spv2[3:6, 0])]

def rodriguesEq(a, th):
    ahat = hat(a)
    return E + ahat*sin(th) + ahat*ahat*(1-cos(th))

def makeRot(axis, th):
    rot = matrix(zeros((3,3)))
    cth = cos(th)
    sth = sin(th)
    if axis == 0:
        rot[0, 0] = 1.0
        rot[1, 1] = cth
        rot[1, 2] = -sth
        rot[2, 1] = sth
        rot[2, 2] = cth
    elif axis == 1:
        rot[0, 0] = cth
        rot[0, 2] = sth
        rot[1, 1] = 1.0
        rot[2, 0] = -sth
        rot[2, 2] = cth
    elif axis == 2:
        rot[0, 0] = cth
        rot[0, 1] = -sth
        rot[1, 0] = sth
        rot[1, 1] = cth
        rot[2, 2] = 1.0
    return rot

def spRot(rot):
    ans = matrix(zeros((6, 6)))
    ans[0:3, 0:3] = rot
    ans[3:6, 3:6] = rot
    return ans

def spXlt(p):
    ans = matrix(E6)
    ans[3:6, 0:3] = -hat(p)
    return ans

def makeX(rot, p):
    ans = spRot(rot) * spXlt(p)
    return ans
