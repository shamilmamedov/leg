#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script implements trajectory tracking in operational space

"""
import can
import numpy as np
import communication as cmctn
import controllers 
import leg_class
import time

# Constants
MOTOR_IDS = [1, 2, 3]

# Trajectory function
def periodic_trajectory(x0, A, omega, t):
    """ computes cartesian position and velocity vectors for a sine trajectory
        x0 [3x1] - offset (position around which we want to achieve oscillations)
        A [3x1] - amplitudes
        omega [3x1] - frequencies
        t - time
    """
    return x0 + np.multiply(A, np.sin(omega*t)), np.multiply(np.multiply(A, omega), np.cos(omega*t))


# Variables to store generalized positions and velocities, and torques
q = np.zeros((3,1))
q_dot = np.zeros((3,1))
tau = np.zeros((3,1))
t = np.empty(0)

# Parameters of the controller
Kp = np.array([[100., 0., 0.], [0., 100., 0.], [0., 0., 100.]])
Kd = 2.5*np.eye(3)
tau_max = 2.0 # saturation torque

# Trajectory parameters
A = np.array([0.1, 0.1, 1.5*0.1])
omega = 2*np.array([np.pi, np.pi, np.pi])


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

x0 = leg.getForwardKinematics(q[:,0])
for i in range(20000):
    t = np.append(t, time.clock())
    x_des_i, x_dot_des_i = periodic_trajectory(x0, A, omega, t[i] - t[0])

    # Change reference positions and velocities
    controller.change_reference(x_des_i, x_dot_des_i)

    # compute cartesian position and velocity
    xi = leg.getForwardKinematics(q[:,i])
    Ji = leg.getJacobian(q[:,i])[0:3,0:3]
    xdoti = np.dot(Ji, q_dot[:,i])

    # PD control of the chosen motor
    tau_i = controller.compute_torques(xi, xdoti, Ji)

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