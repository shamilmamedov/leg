#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import time
from matplotlib import pyplot as plt

thigh_length = 0.208
shin_length = 0.22

Tpj = np.zeros((2,4,4))
Tpj[0,:,:] = [[1., 0., 0., -thigh_length],
              [0., 1., 0., 0.], 
              [0., 0., 1., 0.],
              [0., 0., 0., 1.]]
Tpj[1,:,:] = [[1., 0., 0., -shin_length],
              [0., 1., 0., 0.], 
              [0., 0., 1., 0.],
              [0., 0., 0., 1.]]

def rot_z(q):
    return np.array([[np.cos(q), -np.sin(q), 0.], 
                     [np.sin(q), np.cos(q), 0.],
                     [0., 0., 1.]])

def RpToTrans(R, p):
        """Convert a rotation matrix and a position vector into
        homogenous transformation matrix """
        return np.r_[np.c_[R, p], [[0., 0., 0., 1.]]]

def TransToRp(T):
        """ Convertes a homogenous transformation matrix into a rotation matrix
        and position vector  """
        return T[0:3, 0:3], T[0:3, 3]


def get_forward_kinematics(q):
    """ computes forward kinematics of the hopper.
    It is assumed that the world frame is located
    at the origin of the first joint """
    # rotation of the first joint
    Rw1 = rot_z(q[0])
    Tw1 = RpToTrans(Rw1, np.zeros(3))

    R22 = rot_z(q[1])
    T12 = np.dot(Tpj[0,:,:], RpToTrans(R22, np.zeros(3)))

    Twee = np.dot(Tw1, np.dot(T12, Tpj[1,:,:]))
    return TransToRp(Twee)[1][0:2]


def get_jacobian(q):
    """ computes translational part of the jacobian of the hopper """
    Rw1 = rot_z(q[0])
    Tw1 = RpToTrans(Rw1, np.zeros(3))

    R22 = rot_z(q[1])
    T12 = np.dot(Tpj[0,:,:], RpToTrans(R22, np.zeros(3)))

    Tw2 =  np.dot(Tw1, T12)
    Twee = np.dot(Tw2, Tpj[1,:,:])

    Jv = np.zeros((3,2))
    Jv[:,0] = np.cross([0, 0, 1], TransToRp(Twee)[1] - TransToRp(Tw1)[1])
    Jv[:,1] = np.cross([0, 0, 1], TransToRp(Twee)[1] - TransToRp(Tw2)[1])
    return Jv[0:2,:]


def get_inverse_kinematics(pos_ee):
    cosq2 = (pos_ee[0]**2 + pos_ee[1]**2 - thigh_length**2 - shin_length**2)/(2*thigh_length*shin_length)
    q2 = np.arctan2(-np.sqrt(1. - cosq2**2), cosq2)
    # q2 = - np.arccos(cosq2)
    q1 = np.arctan2(-pos_ee[1], -pos_ee[0]) - np.arctan2(shin_length*np.sin(q2), thigh_length + shin_length*np.cos(q2))
    return np.array([q1, q2])

def periodic_trajectory(x0, A, omega, t):
    """ computes cartesian position and velocity vectors for a sine trajectory
        x0 [2x1] - offset (position around which we want to achieve oscillations)
        A [2x1] - amplitudes
        omega [2x1] - frequencies
        t - time
    """
    return x0 + np.multiply(A, np.sin(omega*t)) #, np.multiply(np.multiply(A, omega), np.cos(omega*t))


if __name__ == "__main__":
    # q = -np.pi + 2*np.pi*np.random.rand(2)
    q = np.array([0., -0.5])
    print('q = ', q)
    print('fk = ', get_forward_kinematics(q))
    print('ik = ', get_inverse_kinematics(get_forward_kinematics(q)))
    
    x0 = np.array([-0.35, 0.1])
    A = np.array([0.05, 0.])
    omega = np.array([np.pi, 0.])
    
    x = np.zeros((2,1))
    x[:,0] = x0
    t = np.array([time.time()])
    while time.time() - t[0] < 3.:
        t = np.append(t, time.time())
        x = np.append(x, periodic_trajectory(x0, A, omega, time.time() - t[0]).reshape((2,1)), axis=1)

    
    # plt.figure()
    # plt.plot(t, x[0,:])
    # plt.plot(t, x[1,:])
    # plt.show()
        
    for k in range(np.shape(x)[1]):
        print(get_inverse_kinematics(x[:,k]))

    #print(get_jacobian(q))
