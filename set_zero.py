#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 10:22:08 2020

The script is used to set zero positin of the robot
"""
import can
import communication as cmctn


# Constants
MOTOR_ID = 3


# create a bus instance
bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)


# Set given position of the motor to zero
msg_out = can.Message(arbitration_id = MOTOR_ID, data = cmctn.SET_CURRENT_POSITION_TO_ZERO_CODE,
                      is_extended_id=False)
bus.send(msg_out)

# Read incoming message to clear the bus
msg_in = bus.recv()


drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in)

print(pos)

# set current position of the first motor to zero
#msg_set_zero = can.Message(arbitration_id = 1, data = SET_ZERO_POSITION,
#                           is_extended_id=False)
#bus.send(msg_set_zero)