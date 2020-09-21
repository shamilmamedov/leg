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
    """ computes position and velocity vectors for a sin trajectory
        q0 [3x1] - offset
        A [3x1] - amplitudes
        omega [3x1] - frequencies
        t - time
    """
    return q0 + np.multiply(A, np.sin(omega*t)), np.multiply(np.multiply(A, omega), np.cos(omega*t))


# create a bus instance
bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)


# variables to store generalized positions and velocities, and torques
q = np.zeros((3,1))
q_dot = np.zeros((3,1))
tau = np.zeros((3,1))
t = np.empty(0)

q_des = np.array([[],[],[]])
q_dot_des = np.array([[],[],[]])


# parameters of the controller
Kp = 20.*np.eye(3)
Kd = 1.*np.eye(3)
tau_max = 2.0 # saturation torque


# Trajectory parameters
A = np.array([0.2, 0.2, 1.5*0.2])
omega = 2*np.array([np.pi, np.pi, np.pi])


# Create controller instance
controller = cntrl.JointSpacePDController(Kp, Kd)


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
for i in range(10000):
    t = np.append(t, time.clock())
    q_des_i, q_dot_des_i = periodic_trajectory(q0, A, omega, t[i] - t[0])
    q_des = np.append(q_des, q_des_i.reshape((3,1)), axis=1)
    q_dot_des = np.append(q_dot_des, q_dot_des_i.reshape((3,1)), axis=1)
    
    # Change reference positions and velocities
    controller.change_reference(q_des_i, q_dot_des_i)
    
    # PD control of the chosen motor
    tau_i = controller.compute_torques(q[:,i], q_dot[:,i])
    
    # Saturate torques if necessary
    tau_i = cntrl.torque_saturation(tau_i, tau_max)

    # form and send control message
    for k in MOTOR_IDS:
        msg_out_k = can.Message(arbitration_id = k, 
                                data = cmctn.pack_torque_command(tau_i[k-1]),
                                is_extended_id=False)
        bus.send(msg_out_k)
    
    # allocate a place and read incoming messages
    q = np.append(q, np.zeros((3,1)), axis=1)
    q_dot = np.append(q_dot, np.zeros((3,1)), axis=1)
    tau = np.append(tau, np.zeros((3,1)), axis=1)
    for k in range(3):
        msg_in_k = bus.recv()
        drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in_k)
        q[drv_id-1, i+1] = pos
        q_dot[drv_id-1, i+1] = vel
        tau[drv_id-1, i+1] = trq
    
   
# Exit control mode for all motors
for k in MOTOR_IDS:
    msg_k = can.Message(arbitration_id = k, data = cmctn.EXIT_CONTROL_MODE_CODE,
                        is_extended_id=False)
    bus.send(msg_k)

# Read mesaage from the bus 
for k in range(3):
    msg_in_k = bus.recv()
    
    
#%% 
    
plt.figure()
plt.plot(t, q_des[0,:], label='desired')
plt.plot(t, q[0,1:], label='real')
plt.grid()
plt.legend()