import numpy as np
import gym
import featherstone

class MultiLinkEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
    }

    def __init__(self, nlink=3):
        self.mass = 0.5 * np.ones(nlink+1)
        self.llen = 1.0 * np.ones(nlink+1) # length of links
        self.lgc = llen/2.0  # center of mass
        self.jtype = np.zeros(nlink)  # types of joints
        self.lmd = np.array([i for i in range(-1, nlink-1)])
        self.inertia = [np.mat(np.zeros((6, 6))) for i in range(nlink+1)]
        for i in range(nlink+1):
            self.inertia[i][1,1] =(self.mass[i] * self.llen[i]**2)/12.0
            self.inertia[i][2,2] =(self.mass[i] * self.llen[i]**2)/12.0
            self.inertia[i][3:6,3:6] = self.mass[i] * E3
            htmp = hat(np.mat([[lgc[i]], [0.0], [0.0]]))
            self.inertia[i][0:3,0:3] = self,inertia[i][0:3,0:3] + htmp*htmp.T
            self.inertia[i][0:3,3:6] = self.mass[i] * htmp
            self.inertia[i][3:6,0:3] = self.mass[i] * htmp.T
        self.q = np.mat(np.zeros((nlink, 1)))
        self.qdot = np.mat(np.zeros((nlink, 1)))
        self.q2dot = np.mat(np.zeros((nlink, 1)))

        self.fx = [np.mat(np.zeros((6, 1))) for i in range(nlink)]
        self.tau = [np.mat(np.zeros((6, 1))) for i in range(nlink)]
        self.p = [np.mat(np.zeros((3, 1)))] + [np.mat([[llen[i]],[0.0],[0.0]]) for i in range(1, nlink+1)]
        self.Xt = [np.mat(E6)] + [sp_xlt(p[i]) for i in range(1, nlink+1)]
        self.i_X_o  = [np.mat(E6) for i in range(nlink)]

    def _step(self):
        
