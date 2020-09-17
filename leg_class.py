#!/usr/bin/env python
import numpy as np
from urdf_parser_py.urdf import URDF


class legKinematics():
    def __init__(self):
        urdf_file = open("/home/shamil/ros/leg_ws/src/leg_description/urdf/leg_setup.urdf", "r")
        self.urdf = URDF.from_xml_string(urdf_file.read())
        self.world_to_base_joint_no = 4
        self.Twb = self.getWorldToBaseTransform()
        self.Tpj = self.getParentToJointTransforms()


    def Rot(self, q, axis):
        """Computes rotation matrix for a given angle and axis """
        if(axis[0] == 1 and axis[1] == 0 and axis[2] == 0):
            return np.array([[1., 0., 0.],
                             [0., np.cos(q), -np.sin(q)],
                             [0., np.sin(q), np.cos(q)]])
        elif(axis[0] == 0 and axis[1] == 1 and axis[2] == 0):
            return np.array([[np.cos(q), 0., np.sin(q)],
                             [0., 1., 0.],
                             [-np.sin(q), 0., np.cos(q)]])
        elif(axis[0] == 0 and axis[1] == 0 and axis[2] == 1):
            return np.array([[np.cos(q), -np.sin(q), 0.],
                             [np.sin(q), np.cos(q), 0.],
                             [0., 0., 1.]])
        else:
            raise ValueError

    def rpyToR(self, rpy_angles):
        """ Converts raw pitch yaw angles into a rotation matrix"""
        _cA = np.cos(rpy_angles[2])
        _sA = np.sin(rpy_angles[2])
        _cB = np.cos(rpy_angles[1])
        _sB = np.sin(rpy_angles[1])
        _cG = np.cos(rpy_angles[0])
        _sG = np.sin(rpy_angles[0])
        return np.array([[_cA*_cB, _cA*_sB*_sG - _sA*_cG, _cA*_sB*_cG + _sA*_sG],
                         [_sA*_cB, _sA*_sB*_sG + _cA*_cG, _sA*_sB*_cG - _cA*_sG],
                         [-_sB, _cB*_sG, _cB*_cG]])

    def RpToTrans(self, R, p):
        """Convert a rotation matrix and a position vector into
        homogenous transformation matrix """
        return np.r_[np.c_[R, p], [[0., 0., 0., 1.]]]

    def TransToRp(self, T):
        """ Convertes a homogenous transformation matrix into a rotation matrix
        and position vector  """
        return T[0:3, 0:3], T[0:3, 3]

    def getWorldToBaseTransform(self):
        """ Compute transformation from world frame to base of the robot """
        rpy = self.urdf.joints[self.world_to_base_joint_no].origin.rpy
        p = self.urdf.joints[self.world_to_base_joint_no].origin.xyz
        return self.RpToTrans(self.rpyToR(rpy), p)

    def getParentToJointTransforms(self):
        """ Compute constant transformations from parent link to joint """
        Tpj = np.zeros((4,4,4))
        for k in range(4):
            rpy_k = self.urdf.joints[k].origin.rpy
            p_k = self.urdf.joints[k].origin.xyz
            Tpj[k,:,:] = self.RpToTrans(self.rpyToR(rpy_k), p_k)
        return Tpj

    def getForwardKinematics(self, q):
        """Compute forward kinematics """
        Twk = self.Twb
        for k in range(4):
            if(self.urdf.joints[k].type == "fixed"):
                Tjc = np.eye(4)
            else:
                Rjk = self.Rot(q[k], self.urdf.joints[k].axis)
                pjk = np.zeros(3)
                Tjc = self.RpToTrans(Rjk, pjk)
            Tpc = np.dot(self.Tpj[k,:,:], Tjc)

            Twk = np.dot(Twk, Tpc)
        return self.TransToRp(Twk)[1]

    def getJacobian(self, q):
        """ Compute Jacobian"""
        Twk = np.zeros((5,4,4))
        Twk[0,:,:] = self.Twb
        for k in range(4):
            if(self.urdf.joints[k].type == "fixed"):
                Tjc = np.eye(4)
            else:
                Rjk = self.Rot(q[k], self.urdf.joints[k].axis)
                pjk = np.zeros(3)
                Tjc = self.RpToTrans(Rjk, pjk)
            Tpc = np.dot(self.Tpj[k,:,:], Tjc)
            Twk[k+1,:,:] = np.dot(Twk[k,:,:], Tpc)

        # Compute jacobians
        Jv_wk = np.zeros((3,3))
        Jw_wk = np.zeros((3,3))
        for k in range(3):
            joint_axis_wk = np.dot(self.TransToRp(Twk[k+1,:, :])[0], self.urdf.joints[k].axis)
            Jv_wk[:,k] = np.cross(joint_axis_wk, self.TransToRp(Twk[4,:,:])[1] - self.TransToRp(Twk[k+1,:,:])[1])
            Jw_wk[:,k] = joint_axis_wk
        return np.r_[Jv_wk, Jw_wk]


    def RNEA(self, q, q_dot, q_2dot, g):
        # g_ww = np.array([0., 0., g])
        g_ww = np.dot(np.transpose(self.TransToRp(self.Twb)[0]), np.array([0., 0., g]))
        w_kk = np.zeros((3, 4))
        w_dot_kk = np.zeros((3, 4))
        a_kk = np.zeros((3,4))
        ac_kk = np.zeros((3,3))
        Rpc = np.zeros((4,3,3))

        a_kk[:,0] = - g_ww

        for k in range(3):
            Rjk = self.Rot(q[k], self.urdf.joints[k].axis)
            Rpc[k,:,:] = np.dot(self.TransToRp(self.Tpj[k,:,:])[0], Rjk)

            w_kk[:,k+1] = np.dot(np.transpose(Rpc[k,:,:]), w_kk[:,k])  + np.multiply(q_dot[k], self.urdf.joints[k].axis)

            w_dot_kk[:,k+1] = np.dot(np.transpose(Rpc[k,:,:]), w_dot_kk[:,k]) + np.multiply(q_2dot[k], self.urdf.joints[k].axis) + \
                                np.cross(q_dot[k]*w_kk[:,k+1], self.urdf.joints[k].axis)

            _t1 = a_kk[:,k] + np.cross(w_dot_kk[:,k], self.urdf.joints[k].origin.xyz) + \
                    np.cross(w_kk[:,k], np.cross(w_kk[:,k], self.urdf.joints[k].origin.xyz))
            a_kk[:,k+1] = np.dot(np.transpose(Rpc[k,:,:]), _t1)

            _t2 = np.cross(w_dot_kk[:,k+1], self.urdf.links[k+1].inertial.origin.xyz) + \
                    np.cross(w_kk[:,k+1], np.cross(w_kk[:,k+1], self.urdf.links[k+1].inertial.origin.xyz))
            ac_kk[:,k] = a_kk[:,k+1] + _t2

        Rpc[k+1,:,:] = self.TransToRp(self.Tpj[k+1,:,:])[0]


        f_kk = np.zeros((3,4))
        mu_kk = np.zeros((3,4))
        tau = np.zeros(3)
        for k in reversed(range(3)):
            f_kk[:,k] = np.dot(Rpc[k+1,:,:], f_kk[:,k+1]) + np.multiply(self.urdf.links[k+1].inertial.mass, ac_kk[:,k])

            _t3 = -np.cross(f_kk[:,k], self.urdf.links[k+1].inertial.origin.xyz) + \
                    np.cross(np.dot(Rpc[k+1,:,:], f_kk[:,k+1]), \
                    -np.array(self.urdf.joints[k+1].origin.xyz) + np.array(self.urdf.links[k+1].inertial.origin.xyz)) + \
                    np.dot(np.array(self.urdf.links[k+1].inertial.inertia.to_matrix()), w_dot_kk[:,k+1]) + \
                    np.cross(w_kk[:,k+1], np.dot(np.array(self.urdf.links[k+1].inertial.inertia.to_matrix()), w_kk[:,k+1]))
            mu_kk[:,k] = np.dot(Rpc[k+1,:,:], mu_kk[:,k+1]) + _t3
            tau[k] = np.dot(mu_kk[:,k], self.urdf.joints[k].axis)
        return tau





if __name__ == "__main__":
    # Test the class
    leg1 = legKinematics()
    q = np.array([1, 2, 3])
    print(leg1.getForwardKinematics(q))
    print(leg1.getJacobian(q))
    print(leg1.urdf.links[0].inertial.inertia.to_matrix())
    
    
    g = -9.81
    q_dot = np.array([1., 1., 1.])
    q_2dot = np.array([2., 2., 2.])
    
    print(leg1.RNEA(q, q_dot, q_2dot, g))
    
  