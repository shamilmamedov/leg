#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The script implements regulation -- setpoint control
"""
import can
import numpy as np
import matplotlib.pyplot as plt
import communication as cmctn
import controllers as cntrl
import time

# Constants
MOTOR_IDS = [1, 2, 3]


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
controller = cntrl.JointSpacePDController(Kp, Kd)


# Set all the motors to control mode
for k in MOTOR_IDS:
    msg_k = can.Message(arbitration_id = k, data = cmctn.ENTER_CONTROL_MODE_CODE,
                        is_extended_id=False)
    bus.send(msg_k)

# Read initial states 
for k in range(3):
    msg_in_k = bus.recv()
    drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in_k)
    q[drv_id-1, 0] = pos
    q_dot[drv_id-1, 0] = vel
    tau[drv_id-1, 0] = trq
    
"""
q_des = q[:,0] + 0.1 # set desired position
controller.change_reference(q_des)
for i in range(30000):
    
    # PD control of the chosen motor
    tau_i = controller.compute_torques(q[:,i], q_dot[:,i])

    # Saturate torques if necessary
    tau_i = cntrl.torque_saturation(tau_i, tau_max)
    
    for k in MOTOR_IDS:
        msg_out_k = can.Message(arbitration_id = k, 
                                data = cmctn.pack_torque_command(tau_i[k-1]),
                                is_extended_id=False)
        bus.send(msg_out_k)
    
    t = np.append(t, time.clock())
    q = np.append(q, np.zeros((3,1)), axis=1)
    q_dot = np.append(q_dot, np.zeros((3,1)), axis=1)
    tau = np.append(tau, np.zeros((3,1)), axis=1)
    for k in range(3):
        msg_in_k = bus.recv()
        drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in_k)
        q[drv_id-1, i+1] = pos
        q_dot[drv_id-1, i+1] = vel
        tau[drv_id-1, i+1] = trq

"""    

# Exit control mode for all motors
for k in MOTOR_IDS:
    msg_k = can.Message(arbitration_id = k, data = cmctn.EXIT_CONTROL_MODE_CODE,
                        is_extended_id=False)
    bus.send(msg_k)

# Read mesaage from the bus 
for k in range(3):
    msg_in_k = bus.recv()

    

#%% Plot Torques
    """
plt.figure()
plt.plot(t - t[0], tau[0,1:], label = 'tau_1')
plt.plot(t - t[0], tau[1,1:], label = 'tau_2')
plt.plot(t - t[0], tau[2,1:], label = 'tau_3')
plt.legend()
plt.xlabel('t, sec')
plt.grid()

"""
   
