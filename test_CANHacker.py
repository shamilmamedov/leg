#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import can
import communication as cmctn

# Constant
MOTOR_ID = 3


# create a bus instance
bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)


# send enter control mode command
msg_out = can.Message(arbitration_id = MOTOR_ID, data = cmctn.ENTER_CONTROL_MODE_CODE,
                      is_extended_id=False)
bus.send(msg_out)


# read incoming message
msg_in = bus.recv(timeout=1)
drv_id, pos, vel, trq = cmctn.unpack_reply(msg_in)

# print position
print('position = ', pos)


# Exit control mode for all motors
msg_out = can.Message(arbitration_id = MOTOR_ID, data = cmctn.EXIT_CONTROL_MODE_CODE,
                      is_extended_id=False)
bus.send(msg_out)

msg_in = bus.recv(timeout=0.1)

