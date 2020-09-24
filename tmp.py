#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import leg_class

leg = leg_class.LegKinematics()

# Compute position of joints at zero configuration in world frame
Twj = np.zeros((4,4,4))
Twj[0,:,:] = np.dot(leg.Twb, leg.Tpj[0,:,:])
for k in range(1,4):
    Twj[k,:,:] = np.dot(Twj[k-1,:,:], leg.Tpj[k,:,:])
    print(Twj[k,:,:])


""" To solve inverse kinematics (ik) we can start by solving  
    it for last two links which can be considered as planar manipulator.
    For that it is necessary to find length of the links in
    YZ plane
"""
a2 = np.linalg.norm(Twj[2,1:3,3] - Twj[1,1:3,3], 2)
a3 = np.linalg.norm(Twj[3,1:3,3] - Twj[2,1:3,3], 2)

print('a2 = ', a2)
print('a3 = ', a3)

""" Given YZ coordinates of the position vector, compute 
    generalized coordinates q2 and q3
"""

# position of the end-effector in the world frame
pos_ee = leg.getForwardKinematics(np.array([0., 0., 0.]))

# find a vector from the origin of the second joint to end-effector joint
pos_ee2 = pos_ee - Twj[1,0:3,3]

y_ee = pos_ee2[1]
z_ee = pos_ee2[2]

cosq3 = (z_ee**2 + y_ee**2 - a2**2 - a3**2)/(2*a2*a3)
q3 = np.arctan2(np.sqrt(1. - cosq3**2), cosq3)
# q3 = np.arccos(cosq3)

q2 = np.arctan2(z_ee, y_ee) - np.arctan2(a3*np.sin(q3), a2 + a3*np.cos(q3))
#alpha = np.arctan2(z_ee, y_ee)
#beta = np.arccos((z_ee**2 + y_ee**2 + a2**2 - a3**2)/ (2*a2*np.sqrt(z_ee**2 + y_ee**2)))
#q2 = alpha - beta
print(q2, q3)

print(leg.getForwardKinematics(np.array([0., q2, q3])))

