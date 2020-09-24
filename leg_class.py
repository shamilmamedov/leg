# #!/usr/bin/env python3

import numpy as np
from urdf_parser_py.urdf import URDF


class LegKinematics():
    def __init__(self):
        urdf_file = open("/home/shamil/ros/leg_ws/src/leg_description/urdf/leg_setup2.urdf", "r")
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

    def getInverseKinematics(self, x):

        return 0

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

        a_kk[:,0] = -g_ww

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



class TestLegKinematics():
    def test_forward_kinematics(self):
        """ Test forward kinematics by computing froward kinematics for four different configurations and compares
            the result with the result obtained by using matlab robotics systems toolbox 
        """
        leg = LegKinematics()
        q = np.array([[0.8407, 0.2543, 0.8143], [-1.3456, 1.6160, 1.5942], [-2.8026, 0.1935, 1.7541], [2.7270, -2.3254, 0.4324]])
        res = np.array([[0.2252, 0.1187, 0.3647], [-0.1739, 0.1032, -0.1728], [-0.1402, -0.2119, 0.1808], [-0.2022, 0.3060, -0.1527]])
        residual = np.zeros(4)
        for k in range(4):
            x_k = leg.getForwardKinematics(q[k,:])
            residual[k] = np.linalg.norm(x_k - res[k,:])
        return residual

    def test_jacobian(self):
        """ Test jacobians """
        leg = LegKinematics()
        q = np.array([[-2.1226, 1.8490, -1.1862], [0.1793, -2.1008, 0.6408], [-1.4893, 0.9681, 1.1889]])
        res = np.zeros((3,6,3))
        res[0,:,:] = np.array([[-0.1229, -0.0975, -0.1477], [0.3175, -0.0600, -0.0909], [0.0000 ,  -0.3348,   -0.1354], 
                                [0.0000, 0.5242, 0.5242], [0.0000, -0.8516, -0.8516], [-1.0000, 0.0000, 0.0000]])
        res[1,:,:] = np.array([[-0.4037, -0.0141, 0.0043], [0.0104, -0.0779, 0.0239], [0.0000, 0.3991, 0.2187],
                                [-0.0000, -0.9840, -0.9840], [0.0000, 0.1783, 0.1783], [-1.0000, 0.0000, 0.0000]])
        res[2,:,:] = np.array([[0.0905, 0.0054, 0.1213], [0.3495, -0.0004, -0.0099], [0.0000, -0.3557, -0.1833], 
                                [-0.0000, -0.0814, -0.0814], [0.0000, -0.9967, -0.9967], [-1.0000, 0.0000, 0.0000]])

        residual = np.zeros(3)
        for k in range(3):
            J_k = leg.getJacobian(q[k,:])
            residual[k] = np.linalg.norm(J_k - res[k,:,:])
        return residual



if __name__ == "__main__":
    # Test the class
    test = TestLegKinematics()
    print('Forward kinematics residual: ', test.test_forward_kinematics())
    
    print('Jacobian residual:', test.test_jacobian())
    # print('Jacobian: \n', leg1.getJacobian(q)[0:3,0:3])
    #print(leg1.urdf.links[0].inertial.inertia.to_matrix())

    leg = LegKinematics()
    print(leg.Tpj)
    #g = -9.81
    #q_dot = np.array([1., 1., 1.])
    #q_2dot = np.array([2., 2., 2.])

    #print(leg1.RNEA(q, q_dot, q_2dot, g))
