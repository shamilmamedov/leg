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
from matplotlib import pyplot as plt
import csv


# Constants
MOTOR_IDS = [2, 3]


def periodic_trajectory(x0, A, omega, t):
    """ computes cartesian position and velocity vectors for a sine trajectory
        x0 [2x1] - offset (position around which we want to achieve oscillations)
        A [2x1] - amplitudes
        omega [2x1] - frequencies
        t - time
    """
    return x0 + np.multiply(A, np.sin(omega*t)) #, np.multiply(np.multiply(A, omega), np.cos(omega*t))


# Variables to store generalized positions and velocities, and torques
q = np.zeros((2,1))
q_dot = np.zeros((2,1))
tau = np.zeros((2,1))
t = np.empty(0)

q_des = np.empty((2,1))
tau_des = np.empty((2,1))

# Controller paramters
Kp = np.array([[20., 0.], [0., 20.]])
Kd = np.array([[1, 0.], [0., 1]])
tau_max = [18., 18.] # 8 10

Kp_jump = np.array([[50., 0.], [0., 50.]])
Kd_jump = np.array([[1, 0.], [0., 1]])

# Trajectry paramters
A = np.array([0.1, 0.])
omega = np.array([5*np.pi, 0.]) # maximum was 5

# paramter that defines control or reading
only_read_states = True

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
    no_lost_msg = []

    # set initial position as desired position
    # q0 = q[:, 0]
    # print(q0)
    q_pre_jump = np.array([1.4762, -2.4239])
    q_post_jump = np.array([1.07398, -1.7516])
    q0 = np.array([1.07398, -1.7516])
    x0 = hopper_kinematics.get_forward_kinematics(q0)
    t = np.append(t, time.time())

    
    for i in range(5000):
        # Send desired torques to motors
        try:
            if (i < 1000):
                q_des_i = q_pre_jump
                q_des = np.append(q_des, q_des_i.reshape((2,1)), axis=1)
                # regulation error
                q_tilde_i = q_des_i - q[:,i] 
                # compute control signal accroding to PD control law
                tau_i = np.dot(Kp, q_tilde_i) - np.dot(Kd, q_dot[:,i])
            elif (i > 1000 and i < 1250):
                q_des_i = np.array([0., 0.])
                # regulation error
                q_tilde_i = q_des_i - q[:,i]
                # compute control signal accroding to PD control law
                tau_i = np.dot(Kp_jump, q_tilde_i) - np.dot(Kd_jump, q_dot[:,i])
                # tau_i = np.array([-18., 18.])
            else:
                q_des_i = q_post_jump
                q_des = np.append(q_des, q_des_i.reshape((2,1)), axis=1)
                # regulation error
                q_tilde_i = q_des_i - q[:,i] 
                # compute control signal accroding to PD control law
            tau_i = np.dot(Kp, q_tilde_i) - np.dot(Kd, q_dot[:,i])
            # Saturate torques if necessary
            for k in range(2):
                if abs(tau_i[k]) > tau_max[k]:
                    tau_i[k] = np.sign(tau_i[k])*tau_max[k]
            tau_des = np.append(tau_des, tau_i.reshape((2,1)), axis=1)

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
        except:
            break

        # Read states (positions, velocities and measured torques)
        t = np.append(t, time.time())
        q = np.append(q, np.zeros((2,1)), axis=1)
        q_dot = np.append(q_dot, np.zeros((2,1)), axis=1)
        tau = np.append(tau, np.zeros((2,1)), axis=1)
        for k in MOTOR_IDS:
            msg_in_k = bus.recv(timeout=0.005)
            if msg_in_k is None:
                q[k-2, i+1] = q[k-2, i]
                q_dot[k-2, i+1] = q_dot[k-2, i]
                tau[k-2, i+1] = tau[k-2, i]
                # no_lost_msg = no_lost_msg + 1
                no_lost_msg.append(i)
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
    print('number of lost messages', len(no_lost_msg))

# Exit control mode for all motors
for k in MOTOR_IDS:
    msg_k = can.Message(arbitration_id = k, data = cmctn.EXIT_CONTROL_MODE_CODE,
                        is_extended_id=False)
    bus.send(msg_k)

# Read mesaage from the bus
for k in MOTOR_IDS:
    msg_in_k = bus.recv(timeout=0.01)



# write into file
filename = 'logs/max_jump_to_zero'

with open(filename, 'w', newline='') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    spamwriter.writerow(['time', 'q1', 'q2', 'q1_dot', 'q2_dot', 'tau1', 'tau2', 'tau1_ref', 'tau2_ref'])
    for k in range(np.shape(t)[0]):
        current_line = [t[k], q[0,k], q[1,k], q_dot[0,k], q_dot[1,k], tau[0,k], tau[1,k], tau_des[0,k], tau_des[1,k]]
        spamwriter.writerow(current_line)


