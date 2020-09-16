clear all; close all; clc;
%-------------------------------------------------------------------------
% Loading file from urdf
% ------------------------------------------------------------------------

% ur10 = xml2struct('ur10.urdf');
ur10 = xml2struct('/home/shamil/ros/leg_ws/src/leg_description/urdf/leg.urdf');

%-------------------------------------------------------------------------
% Get parameters from urdf
% ------------------------------------------------------------------------
for i = 1:3
% axis of rotation of a joint i in coordinate system of joint i    
   axis_of_rot = str2num(ur10.robot.joint{i}.axis.Attributes.xyz)';
% mass of link (i+1) because joint i rotates link (i+1) as the numbering of
% links starts from base link that it not moving
   link_mass = str2double(ur10.robot.link{i+1}.inertial.mass.Attributes.value);
% poistion of the com in frame attached to link
   com_pos = str2num(ur10.robot.link{i+1}.inertial.origin.Attributes.xyz)';
   com_vec2mat = vec2skewSymMat(com_pos);
% inertial parameters of the link expressed in coordinate system attached
% the center of mass.
   ixx = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.ixx);
   ixy = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.ixy);
   ixz = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.ixz);
   iyy = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.iyy);
   iyz = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.iyz);
   izz = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.izz);
% the inertia tensor wrt the frame oriented as the body frame and with the
% origin in the COM
   link_inertia = [ixx, ixy, ixz; ixy, iyy iyz; ixz, iyz, izz];
% manipulator regressor
   ur10.k(:,i) = axis_of_rot;
   ur10.r_com(:,i) = com_pos;
   ur10.m(i) = link_mass;
   ur10.I(:,:,i) = link_inertia;
   ur10.h(:,i) = link_mass*com_pos;
%    ur10.I_vec(:,i) = inertiaMatrix2Vector(link_inertia-...
%                             link_mass*com_vec2mat*com_vec2mat);
%    ur10.pi(:,i) = [ur10.I_vec(:,i);ur10.h(:,i);ur10.m(i)];
end

% q = pi*rand(3,1);
q = [1, 2, 3]';
qd = zeros(3,1);
q2d = zeros(3,1);
% ------------------------------------------------------------------------
% Lagrangian Formulation of Dynamics
% ------------------------------------------------------------------------
M = zeros(3,3);
G = zeros(3,1);
T_0k(:,:,1) = eye(4);
Jv_0k = zeros(3,3,3);
Jw_0k = zeros(3,3,3);
for i = 1:3
%       Transformation from parent link frame p to current joint frame
        R_pj = RPY(str2num(ur10.robot.joint{i}.origin.Attributes.rpy));
        p_pj = str2num(ur10.robot.joint{i}.origin.Attributes.xyz)';
        T_pj = [R_pj, p_pj; zeros(1,3), 1];
%       Tranformation from joint frame of the joint that rotaties body k to
%       link frame. The transformation is pure rotation
        R_jk = Rot(q(i),ur10.k(:,i));
        p_jk = zeros(3,1);
        T_jk = [R_jk, p_jk; zeros(1,3),1];
%       Transformation from parent link frame p to current link frame k
        T_pk(:,:,i) = T_pj*T_jk;
        T_0k(:,:,i+1) = T_0k(:,:,i)*T_pk(:,:,i);
        z_0k(:,i) = T_0k(1:3,1:3,i+1)*ur10.k(:,i);
        
        r_0k(:,i) = [eye(3),zeros(3,1)]*T_0k(:,:,i+1)*[ur10.r_com(:,i);1];
        
        for j = 1:i
           Jv_0k(:,j,i) = cross(z_0k(:,j),r_0k(:,i)-T_0k(1:3,4,j+1));
           Jw_0k(:,j,i) = z_0k(:,j);
        end
        
        M = M + ur10.m(i)*(Jv_0k(:,:,i)'*Jv_0k(:,:,i)) + ...
            Jw_0k(:,:,i)'*T_0k(1:3,1:3,i+1)*ur10.I(:,:,i)*...
            T_0k(1:3,1:3,i+1)'*Jw_0k(:,:,i);
        G = G + Jv_0k(:,:,i)'*ur10.m(i)*[0;0;9.81];
end

%%
% ------------------------------------------------------------------------
% Newton-Euler formulation
% ------------------------------------------------------------------------
g_00 = [0;0;-9.81];
w_kk(:,1) = zeros(3,1);
wd_kk(:,1) = zeros(3,1);
a_kk(:,1) = -g_00;

for i = 1:3
%       Transformation from parent link frame p to current joint frame
        R_pj = RPY(str2num(ur10.robot.joint{i}.origin.Attributes.rpy));
        R_jk = Rot(q(i),ur10.k(:,i));
        R_pk(:,:,i) = R_pj*R_jk;

        p_pj = str2num(ur10.robot.joint{i}.origin.Attributes.xyz)';
        
        w_kk(:,i+1) = R_pk(:,:,i)'*w_kk(:,i) + qd(i)*ur10.k(:,i);
        
        wd_kk(:,i+1) = R_pk(:,:,i)'*wd_kk(:,i) + q2d(i)*ur10.k(:,i) + ...
                        cross(qd(i)*w_kk(:,i+1),ur10.k(:,i));
                    
        a_kk(:,i+1) = R_pk(:,:,i)'*(a_kk(:,i) + ...
                        cross(wd_kk(:,i),p_pj) + ...
                        cross(w_kk(:,i),cross(w_kk(:,i),p_pj)));
                                        
        ac_kk(:,i) = a_kk(:,i+1) + cross(wd_kk(:,i+1),ur10.r_com(:,i)) + ...
                        cross(w_kk(:,i+1),cross(w_kk(:,i+1),ur10.r_com(:,i)));    
end
R_pk(:,:,i+1) = RPY(str2num(ur10.robot.joint{i+1}.origin.Attributes.rpy));



f_kk = zeros(3,4);
mu_kk = zeros(3,4);
for i = 3:-1:1
        p_pj = str2num(ur10.robot.joint{i+1}.origin.Attributes.xyz)';
        
        f_kk(:,i) = R_pk(:,:,i+1)*f_kk(:,i+1) + ur10.m(i)*ac_kk(:,i);
        
        mu_kk(:,i) = R_pk(:,:,i+1)*mu_kk(:,i+1) - ...
                        cross(f_kk(:,i),ur10.r_com(:,i)) + ...
                        cross(R_pk(:,:,i+1)*f_kk(:,i+1),-p_pj + ur10.r_com(:,i)) + ...
                        ur10.I(:,:,i)*wd_kk(:,i+1) + ...
                        cross(w_kk(:,i+1),ur10.I(:,:,i)*w_kk(:,i+1));
        tau(i) = mu_kk(:,i)'*ur10.k(:,i); 
end

tau
return
% ------------------------------------------------------------------------
% Testing results
% ------------------------------------------------------------------------
rbt = importrobot('/home/shamil/ros/leg_ws/src/leg_description/urdf/leg.urdf');
rbt.DataFormat = 'column';
rbt.Gravity = [0 0 -9.81];

M_matlab = massMatrix(rbt,q);
G_matlab = gravityTorque(rbt,q);
tau_matlab =  inverseDynamics(rbt,q,qd,q2d)
tau'