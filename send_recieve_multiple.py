#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Send commands and read from multiple motors
"""
import can
import numpy as np
import matplotlib.pyplot as plt
import communication as cmctn

# Constants
MOTOR_IDS = [1, 2, 3]

# create a bus instance
bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)

# set current position of the first motor to zero
#msg_set_zero = can.Message(arbitration_id = 1, data = SET_ZERO_POSITION,
#                           is_extended_id=False)
#bus.send(msg_set_zero)


q = [0]*3
q_dot = [0]*3
tau = [0]*3

# Set all the motors to control mode
for k in MOTOR_IDS:
    msg_k = can.Message(arbitration_id = k, data = cmctn.ENTER_CONTROL_MODE_CODE,
                        is_extended_id=False)
    bus.send(msg_k)

# Read initial states 
for k in range(3):
    msg_in_k = bus.recv()
    drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in_k)
    q[drv_id-1] = pos
    q_dot[drv_id-1] = vel
    tau[drv_id-1] = trq
    

pos_vec = []
vel_vec = []
trq_vec = []
time_vec = []


TEST_MOTOR_ID = 1
for i in range(10000):
    t_ff = 0.0
    
    msg = can.Message(arbitration_id = TEST_MOTOR_ID,
                      data = cmctn.pack_torque_command(t_ff),
                      is_extended_id = False)
    bus.send(msg)

    msg_in = bus.recv()
    drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in)
    pos_vec.append(pos)
    vel_vec.append(vel)
    trq_vec.append(trq)
    time_vec.append(msg_in.timestamp)
    
    

    
#%%
time_vec = np.array(time_vec)

fig, axs = plt.subplots(3,1)

axs[0].plot(time_vec-time_vec[0], pos_vec)
axs[0].set_xlabel('time, s')
axs[0].set_ylabel('position')
axs[0].grid(True)

axs[1].plot(time_vec-time_vec[0], vel_vec)
axs[1].set_xlabel('time, s')
axs[1].set_ylabel('velocity')
axs[1].grid(True)

axs[2].plot(time_vec-time_vec[0], trq_vec)
axs[2].set_xlabel('time, s')
axs[2].set_ylabel('torque')
axs[2].grid(True)
