#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This module is all about hopper...
"""
import can
import numpy as np
import time
import communication as cmctn
import hopper_kinematics


# Constants
MOTOR_IDS = [2, 3]

# Variables to store generalized positions and velocities, and torques
q = np.zeros((2,1))
q_dot = np.zeros((2,1))
tau = np.zeros((2,1))
t = np.empty(0)

# Controller paramters
Kp = np.array([[20., 0.], [0., 20]])
Kd = np.array([[1, 0.], [0., 1]])
tau_max = 4.

# paramter that defines control or reading
only_read_states = False

# Create a bus instance
bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)


# Set all the motors to control mode
for k in MOTOR_IDS:
    msg_k = can.Message(arbitration_id = k, data = cmctn.ENTER_CONTROL_MODE_CODE,
                        is_extended_id=False)
    bus.send(msg_k)

# Read initial states
for k in MOTOR_IDS:
    msg_in_k = bus.recv()
    drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in_k)
    if k == 3:
        q[drv_id-2, 0] = pos/1.5
        q_dot[drv_id-2, 0] = vel/1.5
        tau[drv_id-2, 0] = trq*1.5    
    else:
        q[drv_id-2, 0] = pos
        q_dot[drv_id-2, 0] = vel
        tau[drv_id-2, 0] = trq

if only_read_states:
    print('Initial position = ', q[:,0])
else:
    # number of packages lost
    no_lost_msg = 0

    # set initial position as desired position
    q_des = q[:, 0]
    x_des = hopper_kinematics.get_forward_kinematics(q[:,0])

    for i in range(1000000):
        # compute cartesian position, velocity and linear jacobian
        xi = hopper_kinematics.get_forward_kinematics(q[:,i])
        Ji = hopper_kinematics.get_jacobian(q[:,i])
        xdoti =  np.dot(Ji, q_dot[:,i])
        

        # regulation error
        x_tilde_i = x_des - xi
        q_tilde_i = q_des - q[:,i] 
        # compute control signal accroding to PD control law
        # tau_i = np.dot(np.transpose(Ji), np.dot(Kp, x_tilde_i)) - np.dot(np.transpose(Ji), np.dot(Kd, xdoti))
        tau_i = np.dot(Kp, q_tilde_i) - np.dot(Kd, q_dot[:,i])
        # Saturate torques if necessary
        for k in range(2):
            if abs(tau_i[k]) > tau_max:
                tau_i[k] = np.sign(tau_i[k])*tau_max

        # Send desired torques to motors
        for k in MOTOR_IDS:
            if k == 3:
                msg_out_k = can.Message(arbitration_id = k,
                                        data = cmctn.pack_torque_command(tau_i[k-2]/1.5),
                                        is_extended_id=False)
            else:
                msg_out_k = can.Message(arbitration_id = k,
                                        data = cmctn.pack_torque_command(tau_i[k-2]),
                                        is_extended_id=False)
            bus.send(msg_out_k)

        # time.sleep(0.005)

        # Read states (positions, velocities and measured torques)
        t = np.append(t, time.clock())
        q = np.append(q, np.zeros((2,1)), axis=1)
        q_dot = np.append(q_dot, np.zeros((2,1)), axis=1)
        tau = np.append(tau, np.zeros((2,1)), axis=1)
        for k in MOTOR_IDS:
            msg_in_k = bus.recv(timeout=0.005)
            if msg_in_k is None:
                q[drv_id-2, i+1] = q[drv_id-2, i]
                q_dot[drv_id-2, i+1] = q_dot[drv_id-2, i]
                tau[drv_id-2, i+1] = tau[drv_id-2, i]
                no_lost_msg = no_lost_msg + 1
            else:
                drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in_k)
                if k == 3:
                    q[drv_id-2, i+1] = pos/1.5
                    q_dot[drv_id-2, i+1] = vel/1.5
                    tau[drv_id-2, i+1] = trq*1.5
                else:
                    q[drv_id-2, i+1] = pos
                    q_dot[drv_id-2, i+1] = vel
                    tau[drv_id-2, i+1] = trq
    print('number of lost messages', no_lost_msg)

# Exit control mode for all motors
for k in MOTOR_IDS:
    msg_k = can.Message(arbitration_id = k, data = cmctn.EXIT_CONTROL_MODE_CODE,
                        is_extended_id=False)
    bus.send(msg_k)

# Read mesaage from the bus
for k in MOTOR_IDS:
    msg_in_k = bus.recv(timeout=0.01)
