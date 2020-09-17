#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 17 10:10:48 2020

module contains classes that implement different controllers
"""
import numpy as np

class jointSpacePDController():
    def __init__(self, Kp, Kd, qd = np.zeros(3)):
        self.Kp = Kp
        self.Kd = Kd
        self.qd = qd # desired position

    def changeSetpoint(self, qd):
        self.qd = qd

    def computeJointTorques(self, q, q_dot):
        """ !!! Correct algorithm should implement gravity compensation """
        return np.dot(self.Kp, self.qd - q) - np.dot(self.Kd, q_dot)

   
    
class  operationalSpacePDController():
    def __init__(self, Kp, Kd, xd = np.zeros(3)):
        self.Kp = Kp
        self.Kd = Kd
        self.xd = xd # desired cartesian position

    def changeSetpoint(self, xd):
        self.xd = xd

    def computeJointTorques(self, x, q_dot, J):
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
    controller = jointSpacePDController(Kp, Kd, qd)
    
    # Mmake sure that it is possible to change setpoint
    print('Old setpoint: ', controller.qd)
    controller.changeSetpoint(np.array([1, 1, 1]))
    print('New setpoint: ', controller.qd)
    
    # Compute control torques
    q = np.zeros(3)
    q_dot = np.array([1, 1, 1])
    print('Control Torques: ', controller.computeJointTorques(q, q_dot))
   
    print('\n')
    # Test operation space pd controller
    xd = np.array([0., 0., 0.2])
    controller2 = operationalSpacePDController(Kp, Kd, xd)
    
    # Test that it is possible to change setpoint
    print('Old setpoint: ', controller2.xd)
    xd_new = np.array([0., 0., 0.25])
    controller2.changeSetpoint(xd_new)
    print('New setpoint: ', controller2.xd)
    
    # Test computation of the control torques
    x = np.array([0., 0., 0.15])
    q_dot = np.array([0., 0., 0.])
    J = np.random.rand(3)
          
    print('Control Torques: ', controller2.computeJointTorques(x, q_dot, J))
    
    