#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 15:03:13 2020

Trajectory tracking for all joints
"""
import can
import numpy as np
import matplotlib.pyplot as plt
import communication as cmctn
import controllers as cntrl
import time

# Constants
MOTOR_IDS = [1, 2, 3]

# Trajectory function
def periodic_trajectory(q0, A, omega, t):
    return q0 + np.multiply(A, np.sin(omega*t)), np.multiply(np.multiply(A, omega), np.cos(omega*t))

# create a bus instance
bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)

# variables to store generalized positions and velocities, and torques
q = np.zeros((3,1))
q_dot = np.zeros((3,1))
tau = np.zeros((3,1))
t = np.empty(0)

# parameters of the controller
Kp = 10.*np.eye(3)
Kd = 1.*np.eye(3)
tau_max = 2.0 # saturation torque

# Create controller instance
controller = cntrl.jointSpacePDController(Kp, Kd)

# Set all the motors to control mode
for k in MOTOR_IDS:
    msg_out_k = can.Message(arbitration_id = k, data = cmctn.ENTER_CONTROL_MODE_CODE,
                            is_extended_id=False)
    bus.send(msg_out_k)
    
# Read initial states 
for k in range(3):
    msg_in_k = bus.recv()
    drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in_k)
    q[drv_id-1, 0] = pos
    q_dot[drv_id-1, 0] = vel
    tau[drv_id-1, 0] = trq
    
q0 = q[:,0]
