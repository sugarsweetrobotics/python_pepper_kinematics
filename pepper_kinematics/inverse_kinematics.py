import math
import numpy as np
import scipy as sp
from scipy import linalg

import forward_kinematics as fk

def calc_inv_pos(angles, target_pos, target_ori, epsilon, right=True):
    p  = np.array([0,0,0,1])
    angs = np.array([a for a in angles])
    sum_old = 100000
    while True:
        pos, ori, j = fk.calc_fk_and_jacob(angs, jacob=True, right=right)
        J = _calc_invJ(j)
        delta_pos = np.matrix((target_pos-pos)[0:3]).transpose()
        v = (J * delta_pos).transpose()
        angs = np.squeeze(np.asarray(v)) + angs
        
        sum = 0
        for d in delta_pos:
            sum = sum + math.fabs(d)
        #sum = np.sum(delta_pos)
        if sum < epsilon:
            break
        if sum > sum_old:
            print '# set_position error : Distance can not converged.'
            return None
        sum_old = sum
    return angs

def _calc_invJ(J, epsilon = 0.01):
    u, sigma, v = np.linalg.svd(J, full_matrices=0)
    sigma_ = [1/s if s > epsilon else 0 for s in sigma]
    rank = np.shape(J)[0]
    return np.matrix(v.transpose()) * np.matrix(linalg.diagsvd(sigma_, rank, rank)) * np.matrix(u.transpose())
