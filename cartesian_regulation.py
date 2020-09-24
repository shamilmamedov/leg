#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 22 07:25:42 2020

@author: shamil

In this script we implement cartesian PD control, more specifically
regulation (setpoint control)
"""
import can
import numpy as np
import communication as cmctn
import controllers 
import leg_class
import time

# Constants
MOTOR_IDS = [1, 2, 3]


# Variables to store generalized positions and velocities, and torques
q = np.zeros((3,1))
q_dot = np.zeros((3,1))
tau = np.zeros((3,1))
t = np.empty(0)

# Parameters of the controller
Kp = np.array([[100., 0., 0.], [0., 100., 0.], [0., 0., 100.]])
Kd = 2.5*np.eye(3)
tau_max = 2.0 # saturation torque

# Create Operational space PD cotnroller instance
controller = controllers.OperationalSpacePDController(Kp, Kd)

# Create leg instance
leg = leg_class.LegKinematics()

# Create a bus instance
bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)



# Set all the motors to control mode
for k in MOTOR_IDS:
    msg_k = can.Message(arbitration_id = k, data = cmctn.ENTER_CONTROL_MODE_CODE,
                        is_extended_id=False)
    bus.send(msg_k)

# Read initial states
for k in range(3):
    msg_in_k = bus.recv()
    drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in_k)
    if k == 2:
        q[drv_id-1, 0] = pos/1.5
        q_dot[drv_id-1, 0] = vel/1.5
        tau[drv_id-1, 0] = trq*1.5    
    else:
        q[drv_id-1, 0] = pos
        q_dot[drv_id-1, 0] = vel
        tau[drv_id-1, 0] = trq

# print(q[:,0])

q_desired = q[:,0] # give some increment to current position
x_desired = leg.getForwardKinematics(q_desired)
controller.change_setpoint(x_desired)
for i in range(20000):
    # compute cartesian position and linear jacobian
    xi = leg.getForwardKinematics(q[:,i])
    Ji = leg.getJacobian(q[:,i])[0:3,0:3]

    # PD control of the chosen motor
    tau_i = controller.compute_torques(xi, q_dot[:,i], Ji)

    # Saturate torques if necessary
    tau_i = controllers.torque_saturation(tau_i, tau_max)

    # Send desired torques to motors
    for k in MOTOR_IDS:
        if k == 3:
            msg_out_k = can.Message(arbitration_id = k,
                                    data = cmctn.pack_torque_command(tau_i[k-1]/1.5),
                                    is_extended_id=False)
        else:
            msg_out_k = can.Message(arbitration_id = k,
                                    data = cmctn.pack_torque_command(tau_i[k-1]),
                                    is_extended_id=False)
        bus.send(msg_out_k)

    # Read states (positions, velocities and measured torques)
    t = np.append(t, time.clock())
    q = np.append(q, np.zeros((3,1)), axis=1)
    q_dot = np.append(q_dot, np.zeros((3,1)), axis=1)
    tau = np.append(tau, np.zeros((3,1)), axis=1)
    for k in range(3):
        msg_in_k = bus.recv()
        drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in_k)
        if k == 2:
            q[drv_id-1, i+1] = pos/1.5
            q_dot[drv_id-1, i+1] = vel/1.5
            tau[drv_id-1, i+1] = trq*1.5
        else:
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

