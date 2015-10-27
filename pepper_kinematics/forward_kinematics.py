import math
import numpy as np
import scipy as sp
from scipy import linalg


L1 = 0.14974
L2 = 0.015
L3 = 0.1812
L4 = 0
L5 = 0.150
L6 = 0.0695
L7 = 0.0303

p = np.array([0,0,0,1])
v0 = np.array([[1],[0],[0],[0]])
v1 = np.array([[0],[1],[0],[0]])
v2 = np.array([[0],[0],[1],[0]])


def transX(th, x, y, z):
    s = math.sin(th)
    c = math.cos(th)
    return np.array([[1, 0, 0, x], [0, c, -s, y], [0, s, c, z], [0, 0, 0, 1]])

def transY(th, x, y, z):
    s = math.sin(th)
    c = math.cos(th)
    return np.array([[c, 0, -s, x], [0, 1, 0, y], [s, 0, c, z], [0, 0, 0, 1]])

def transZ(th, x, y, z):
    s = math.sin(th)
    c = math.cos(th)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def calc_fk_and_jacob(angles, jacob=True, right=True):
    _L1_ = -L1 if right else L1
    _L2_ = -L2 if right else L2

    T1 = transY(-angles[0], 0, _L1_, 0)
    T2 = transZ(angles[1], 0, 0, 0)
    Td = transY(9.0/180.0*math.pi, L3, _L2_, 0)
    T3 = transX(angles[2], 0, 0, 0)
    T4 = transZ(angles[3], 0, 0, 0)
    T5 = transX(angles[4], L5, 0, 0)
    T6 = transZ(0, L6, 0, -L7)
    
    T1Abs = T1
    T2Abs = T1Abs.dot(T2)
    TdAbs = T2Abs.dot(Td)
    T3Abs = TdAbs.dot(T3)
    T4Abs = T3Abs.dot(T4)
    T5Abs = T4Abs.dot(T5)
    T6Abs = T5Abs.dot(T6)

    pos = T6Abs.dot(p)
    ori = T6Abs[0:3,0:3]

    if not jacob:
        return pos, ori

    OfstT1 = L1 * T1Abs.dot(v1)
    OfstTd = TdAbs.dot(np.array([[L3], [L2], [0], [0]]))
    OfstT5 = L5 * T5Abs.dot(v0)
    OfstT6 = T5Abs.dot(np.array([[L6], [0], [-L7], [0]]))

    vec6 = OfstT6
    vec5 = vec6 + OfstT5
    vec4 = vec5
    vec3 = vec4
    vecd = vec3 + OfstTd
    vec2 = vecd
    vec1 = vec2 + OfstT1
    
    j1 = T1Abs.dot(v1)
    j2 = T2Abs.dot(v2)
    jd = TdAbs.dot(v1)
    j3 = T3Abs.dot(v0)
    j4 = T4Abs.dot(v2)
    j5 = T5Abs.dot(v0)
    
    J1 = cross(j1, vec1)
    J2 = cross(j2, vec2)
    J3 = cross(j3, vec3)
    J4 = cross(j4, vec4)
    J5 = cross(j5, vec5)
    
    J = np.c_[J1, J2, J3, J4, J5]
    return pos, ori, J


def cross(j, v):
    t0 = j[1][0] * v[2][0] - j[2][0] * v[1][0]
    t1 = j[2][0] * v[0][0] - j[0][0] * v[2][0]
    t2 = j[0][0] * v[1][0] - j[1][0] * v[0][0]
    return np.array([[t0], [t1], [t2]])
        
