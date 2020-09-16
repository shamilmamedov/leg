#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module for communication with motor through CAN
"""

# Constants
P_MIN, P_MAX = -95.5, 95.5
V_MIN, V_MAX = -30.0, 30.0
T_MIN, T_MAX = -18.0, 18.0

KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

ZERO_POSITION_UINT = 32767
ZERO_VELOCITY_UINT = 2047
ZERO_KP_UINT = 0
ZERO_KD_UINT = 0

ENTER_CONTROL_MODE_CODE = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc]
EXIT_CONTROL_MODE_CODE = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd]
SET_CURRENT_POSITION_TO_ZERO_CODE = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe]


# Conversion functions
def float_to_uint(x,  x_min, x_max, bits):
    """ Convert float to unsigned int"""
    span = x_max - x_min
    offset = x_min
    return int((x-offset)*(float((1<<bits)-1))/span)

def uint_to_float(x_int,  x_min, x_max, bits):
    """Convert unsigned int to float """
    span = x_max - x_min
    offset = x_min
    return (float(x_int))*span/(float((1<<bits)-1)) + offset


def unpack_reply(msg):
    """ unpack the reply message that is sent back by motor  """
    # unpack ints from the can buffer
    driver_id = msg.data[0]
    pos_int = (msg.data[1]<<8)|msg.data[2]
    vel_int = (msg.data[3]<<4)|(msg.data[3]>>4)
    tau_int = ((msg.data[4]&0xF)<<8)|msg.data[5]
    #convert to floats
    pos = uint_to_float(pos_int, P_MIN, P_MAX, 16)
    vel = uint_to_float(vel_int, V_MIN, V_MAX, 12)
    tau = uint_to_float(tau_int, T_MIN, T_MAX, 12)
    return driver_id, pos, vel, tau


def pack_torque_command(tau):
    """ form a data that consists of torque command to motor and zeros for
        position, velocity, Kp and Kd """
    tau_int = float_to_uint(tau, T_MIN, T_MAX, 12)
    data = [0]*8
    data[0] = ZERO_POSITION_UINT>>8
    data[1] = ZERO_POSITION_UINT&0xFF
    data[2] = ZERO_VELOCITY_UINT>>4
    data[3] = ((ZERO_VELOCITY_UINT&0xF)<<4)|(ZERO_KP_UINT>>8)
    data[4] = ZERO_KP_UINT&0xFF
    data[5] = ZERO_KD_UINT>>4
    data[6] = ((ZERO_KD_UINT&0xF)<<4)|(tau_int>>8)
    data[7] = tau_int&0xff
    return data