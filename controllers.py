#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 17 10:10:48 2020

module contains classes that implement different controllers
"""
import numpy as np


# Torque saturation function
def torque_saturation(tau, tau_max): 
    """Function that saturates joint torques by a given maximum value """
    for k in range(3):
        if abs(tau[k]) > tau_max:
            tau[k] = np.sign(tau[k])*tau_max
    return tau


class JointSpacePDController():
    """A class that implements joint space PD controller """
    def __init__(self, Kp, Kd, desired_position=np.zeros(3), desired_velocity=np.zeros(3)):
        self.Kp = Kp
        self.Kd = Kd
        self.desired_position = desired_position 
        self.desired_velocity = desired_velocity 

    def change_reference(self, q, q_dot=np.zeros(3)):
        self.desired_position = q
        self.desired_velocity = q_dot

    def compute_torques(self, q, q_dot):
        """ !!! Correct algorithm should implement gravity compensation """
        return np.dot(self.Kp, self.desired_position - q) \
                + np.dot(self.Kd, self.desired_velocity - q_dot)

   
    
class  OperationalSpacePDController():
    def __init__(self, Kp, Kd, xd = np.zeros(3)):
        self.Kp = Kp
        self.Kd = Kd
        self.xd = xd # desired cartesian position

    def change_setpoint(self, xd):
        self.xd = xd

    def compute_torques(self, x, q_dot, J):
        """ !!! pay attention to input
            :x[3] is cartesian position of the end-effector
            :q_dot[3] joint space velocitis
            :J[3x3] jacobian - linear part """
        _x_tilde = self.xd - x
        _t1 =  np.dot(np.transpose(J), self.Kp)
        _t2 = np.dot(np.transpose(J), np.dot(self.Kd, J))
        return np.dot(_t1, _x_tilde) - np.dot(_t2, q_dot)




if __name__ == "__main__":
    # Test joint space PD controller
    Kp = np.eye(3)
    Kd = 0.1*np.eye(3)
    qd = np.array([1, 1, 0])
    
    # create controller instance
    controller = JointSpacePDController(Kp, Kd, qd)
    
    # Mmake sure that it is possible to change setpoint
    print('Old setpoint: ', controller.qd)
    controller.change_reference(np.array([1, 1, 1]))
    print('New setpoint: ', controller.qd)
    
    # Compute control torques
    q = np.zeros(3)
    q_dot = np.array([1, 1, 1])
    print('Control Torques: ', controller.compute_torques(q, q_dot))
   
    print('\n')
    # Test operation space pd controller
    xd = np.array([0., 0., 0.2])
    controller2 = OperationalSpacePDController(Kp, Kd, xd)
    
    # Test that it is possible to change setpoint
    print('Old setpoint: ', controller2.xd)
    xd_new = np.array([0., 0., 0.25])
    controller2.change_setpoint(xd_new)
    print('New setpoint: ', controller2.xd)
    
    # Test computation of the control torques
    x = np.array([0., 0., 0.15])
    q_dot = np.array([0., 0., 0.])
    J = np.random.rand(3)
          
    print('Control Torques: ', controller2.compute_torques(x, q_dot, J))
    
    