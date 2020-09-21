#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 10:30:24 2020

Trajectory tracking for single joint
"""

import can
import numpy as np
import matplotlib.pyplot as plt
import communication as cmctn
import time

# Constants
MOTOR_ID = 2

def periodic_trajectory(q0, A, omega, t):
    return q0 + A*np.sin(omega*t), A*omega*np.cos(omega*t)

# create a bus instance
bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)

# variables to store generalized positions and velocities, and torques
q = np.zeros(1)
q_dot = np.zeros(1)
tau = np.zeros(1)
t = np.empty(0)

# parameters of the controller
Kp = 50.
Kd = 2.
tau_max = 2.0 # saturation torque

# Trajectory parameters
A = 0.2
omega = 2*2*np.pi



# Set chosen motor to control mode
msg_out = can.Message(arbitration_id = MOTOR_ID, data = cmctn.ENTER_CONTROL_MODE_CODE,
                      is_extended_id=False)
bus.send(msg_out)

# Read initial state
msg_in = bus.recv()
drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in)
q[0] = pos
q_dot[0] = vel
tau[0] = trq

q0 = q[0]
for i in range(10000):
    t = np.append(t, time.clock())
    q_des_i, q_dot_des_i = periodic_trajectory(q0, A, omega, t[i] - t[0])
    
    # PD control of the chosen motor
    tau_i = np.dot(Kp, q_des_i - q[i]) + np.dot(Kd, q_dot_des_i - q_dot[i])
    if abs(tau_i) > tau_max:
        tau_i = np.sign(tau_i)*tau_max

    
    # send control message
    msg_out = can.Message(arbitration_id = MOTOR_ID, 
                          data = cmctn.pack_torque_command(tau_i),
                          is_extended_id=False)
    bus.send(msg_out)
    
    # read  reply message         
    msg_in = bus.recv()
   
    drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in)
    q = np.append(q, pos)
    q_dot = np.append(q_dot, vel)
    tau = np.append(tau, trq)

    
# Set all the motors to control mode
msg_out = can.Message(arbitration_id = MOTOR_ID, data = cmctn.EXIT_CONTROL_MODE_CODE,
                      is_extended_id=False)
bus.send(msg_out)

# Read mesaage to clear the bus 
msg_in = bus.recv()

#%%
plt.figure()
plt.plot(t, tau[1:])
plt.grid()
