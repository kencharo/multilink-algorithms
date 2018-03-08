#!/usr/bin/python
import cupy as np

X, Y, Z = range(3)

ex = np.array([[1.0], [0.0], [0.0]])
ey = np.array([[0.0], [1.0], [0.0]])
ez = np.array([[0.0], [0.0], [1.0]])
E3 = np.identity(3) # 3x3 Identity
E6 = np.identity(6) # 6x6 Identity

def hat(x):
    """
    outer product matrix
    """
    xhat = np.zeros((3, 3))
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
    xhat = np.zeros((6, 6))
    xhat[0:3, 0:3] = hat(x[0:3, 0])
    xhat[3:6, 0:3] = hat(x[3:6, 0])
    xhat[3:6, 3:6] = hat(x[0:3, 0])
    return xhat

def mcross(v1, v2):
    """
    outer product for numpy matrix
    """
    ans = np.zeros((3, 1))
    ans[0, 0] = v1[1, 0] * v2[2, 0] - v1[2, 0] * v2[1, 0]
    ans[1, 0] = v1[2, 0] * v2[0, 0] - v1[0, 0] * v2[2, 0]
    ans[2, 0] = v1[0, 0] * v2[1, 0] - v1[1, 0] * v2[0, 0]
    return ans

def sp_inv(X):
    """
    inverse spatial matrix
    """
    return np.r_[np.c_[X[0:3, 0:3].T, X[0:3, 3:6].T],
                 np.c_[X[3:6, 0:3].T, X[3:6, 3:6].T]]

def sp_dual(X):
    return np.r_[np.c_[X[0:3, 0:3], X[3:6, 0:3]],
                 np.c_[X[0:3, 3:6], X[3:6, 3:6]]]

def sp_dual_inv(X):
    """
    Conjugate inverse of spatial matrix
    """
    return np.r_[np.c_[X[0:3, 0:3].T, X[3:6, 0:3].T],
                 np.c_[X[0:3, 3:6].T, X[3:6, 3:6].T]]

def sp_cross(spv1, spv2):
    """
    Outer product for spatial vectors
    """
    return np.r_[mcross(spv1[0:3, :], spv2[0:3, :]),
                 mcross(spv1[0:3, :], spv2[3:6, :]) + mcross(spv1[3:6, :], spv2[0:3, :])]

def sp_dual_cross(spv1, spv2):
    return np.r_[mcross(spv1[0:3, :], spv2[0:3, :]) + mcross(spv1[0:3, :], spv2[0:3, :]),
                 mcross(spv1[0:3, :], spv2[3:6, :])]

def rodrigues_eq(a, th):
    ahat = hat(a)
    return E + ahat * np.sin(th) + np.matmul(ahat, ahat) * (1-np.cos(th))

def make_rot(axis, th):
    rot = np.zeros((3,3))
    cth = np.cos(th)
    sth = np.sin(th)
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

def sp_rot(rot):
    ans = np.zeros((6, 6))
    ans[0:3, 0:3] = rot
    ans[3:6, 3:6] = rot
    return ans

def sp_xlt(p):
    ans = np.copy(E6)
    ans[3:6, 0:3] = -hat(p)
    return ans
