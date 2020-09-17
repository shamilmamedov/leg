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

q = np.zeros((3,1))
q_dot = np.zeros((3,1))
tau = np.zeros((3,1))

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
    

Kp = 20*np.eye(3)
Kd = 1*np.eye(3)

q_des = q[:,0] + 0.1 # set desired position
for i in range(5000):
#    t_ff = 0.0
    
    # PD control of the chosen motor
    tau_i = np.dot(Kp, q_des - q[:,i]) - np.dot(Kd, q_dot[:,i])
    
    for k in MOTOR_IDS:
        msg_out_k = can.Message(arbitration_id = k, 
                                data = cmctn.pack_torque_command(tau_i[k-1]),
                                is_extended_id=False)
        bus.send(msg_out_k)
    
    q = np.append(q, np.zeros((3,1)), axis=1)
    q_dot = np.append(q_dot, np.zeros((3,1)), axis=1)
    tau = np.append(tau, np.zeros((3,1)), axis=1)
    for k in range(3):
        msg_in_k = bus.recv()
        drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in_k)
        q[drv_id-1, i+1] = pos
        q_dot[drv_id-1, i+1] = vel
        tau[drv_id-1, i+1] = trq
    
#    time_vec.append(msg_in.timestamp)


# Set all the motors to control mode
for k in MOTOR_IDS:
    msg_k = can.Message(arbitration_id = k, data = cmctn.EXIT_CONTROL_MODE_CODE,
                        is_extended_id=False)
    bus.send(msg_k)

# Read mesaage from the bus 
for k in range(3):
    msg_in_k = bus.recv()
    



"""
pos_vec = []
vel_vec = []
trq_vec = []
time_vec = []


TEST_MOTOR_ID = 1
q_des = q[TEST_MOTOR_ID-1, 0] + 0.1 # chose desired position as 0.1 increemnt to initial position
pos = q[TEST_MOTOR_ID-1, 0] # current measured position
vel = q_dot[TEST_MOTOR_ID-1, 0] # current measured velocity
Kp, Kd = 50., 2. # coefficients of the controller
for i in range(5000):
#    t_ff = 0.0
    
    # PD control of the chosen motor
    t_ff = Kp*(q_des - pos) - Kd*vel
       
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
    
# Exit control mode
msg = can.Message(arbitration_id = TEST_MOTOR_ID, data = cmctn.EXIT_CONTROL_MODE_CODE,
                  is_extended_id=False)
bus.send(msg)
msg = bus.recv()
    
#%%
time_vec = np.array(time_vec)

fig, axs = plt.subplots(3,1)

axs[0].plot(time_vec-time_vec[0], pos_vec)
axs[0].plot(time_vec-time_vec[0], q_des*np.ones(np.shape(pos_vec)))
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
"""