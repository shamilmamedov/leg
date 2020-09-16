import can
import numpy as np
import time
from ctypes import *
# import matplotlib.pyplot as plt

P_MIN = -12.5
P_MAX = 12.5
V_MIN = -65.0
V_MAX = 65.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -18.0
T_MAX = 18.0

def float_to_uint(x,  x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    return int((x-offset)*(float((1<<bits)-1))/span)

def uint_to_float(x_int,  x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    return (float(x_int))*span/(float((1<<bits)-1)) + offset

bus = can.interface.Bus(bustype='socketcan',channel='can0',bitrate=1000000)
# Enter control mode
enter_control_mode_msg = can.Message(arbitration_id = 0,
                                     data = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc],
                                     is_extended_id=False)
bus.send(enter_control_mode_msg)
print(bus.recv())
v_ar = []
p_ar = []


tic = time.clock()
for i in range(500):
    p_des = 0.0
    v_des = 0.0
    kp = 0.0
    kd = 0.0
    t_ff = 5

    #/// convert floats to unsigned ints ///
    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)
    # print(p_int, v_int, kp_int, kd_int, t_int)


    msg = can.Message(arbitration_id = 0,
                      data=[p_int>>8,p_int&0xFF, v_int>>4, ((v_int&0xF)<<4)|(kp_int>>8), kp_int&0xFF, kd_int>>4, ((kd_int&0xF)<<4)|(t_int>>8), t_int&0xff],
                      is_extended_id=False)
    bus.send(msg)
    res = bus.recv()
    
    # res_ar = [x for x in res.data]
    # #print(res_ar)

    # arb_id = c_uint16(res_ar[0])
    # p_int = c_uint16((res_ar[1]<<8)|res_ar[2])
    # v_int = c_uint16((res_ar[3]<<4)|(res_ar[4]>>4))
    # i_int = c_uint16(((res_ar[4]&0xF)<<8)|res_ar[5])

    # p = uint_to_float(p_int.value, P_MIN, P_MAX, 16)
    # v = uint_to_float(v_int.value, V_MIN, V_MAX, 12)
    # t = uint_to_float(i_int.value, -T_MAX, T_MAX, 12)
    # p_ar.append(p)
    # v_ar.append(v)
    # print(int(100*p),int(v),int(1000*t))

# plt.plot(p_ar,label="par")
# plt.legend()
# plt.savefig("tst.png")
toc = time.clock()
print(toc - tic)

for i in range(500):
    p_des = 0.0
    v_des = 0.0
    kp = 0.0
    kd = 0.0
    t_ff = 0.0
    #/// convert floats to unsigned ints ///
    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)
    # print(p_int, v_int, kp_int, kd_int, t_int)

    msg = can.Message(arbitration_id = 0,
                      data=[p_int>>8,p_int&0xFF, v_int>>4, ((v_int&0xF)<<4)|(kp_int>>8), kp_int&0xFF, kd_int>>4, ((kd_int&0xF)<<4)|(t_int>>8), t_int&0xff],
                      is_extended_id=False)
    bus.send(msg)
    res = bus.recv()

toc = time.clock()
print(toc - tic)
