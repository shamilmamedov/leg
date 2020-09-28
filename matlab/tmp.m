clc; clear all; close all;

robot = importrobot('/home/shamil/ros/leg_ws/src/leg_description/urdf/leg_setup2.urdf');
robot.DataFormat = 'column';
robot.Gravity = [0, 0, -9.81];

%%
clc
q = [-1.49, 0.66, 0.5]';
Tw1 = getTransform(robot, q, robot.BodyNames{2});
Tw2 = getTransform(robot, q, robot.BodyNames{3});
Tw3 = getTransform(robot, q, robot.BodyNames{4});
Twee = getTransform(robot, q, robot.BodyNames{5});

T3ee = getTransform(robot, q, robot.BodyNames{4}, robot.BodyNames{5});
pw_3ee = Tw3(1:3,1:3)*T3ee(1:3,4)

T23 = getTransform(robot, q, robot.BodyNames{3}, robot.BodyNames{4});
pw_23 = Tw2(1:3,1:3)*T23(1:3,4)

T24 = getTransform(robot, q, robot.BodyNames{3}, robot.BodyNames{5})
pw_24 = Tw2(1:3,1:3)*T24(1:3,4)

a2 = norm(pw_23(2:3),2);
a3 = norm(pw_3ee(2:3),2);


z_ee = pw_24(3);
y_ee = pw_24(2);

cosq3 = (z_ee^2 + y_ee^2 - a2^2 - a3^2)/(2*a2*a3);
q3 = atan2(sqrt(1 - cosq3^2), cosq3)

q2 = atan2(y_ee, z_ee) + atan2(a3*sin(q3), a2 + a3*cos(q3))

return
T_matlab = getTransform(robot, q, robot.BodyNames{5});
J = geometricJacobian(robot, q, robot.BodyNames{5});
pos_matlab = T_matlab(1:3,4);

q'

Jv = J(4:6,:)

Jw = J(1:3,:)

return

%%
q  = [1, 2, 3]';
q_dot = [1, 1, 1]';
q_2dot = [2, 2, 2]';
inverseDynamics(robot,q,q_dot,q_2dot)

leg = xml2struct('/home/shamil/ros/leg_ws/src/leg_description/urdf/leg_setup2.urdf');


return
%% Computing forward kinematics
q = [0.1, 0.5, 0.1]';

% Get transform from world frame to base frame
worl_to_base_joint_no = 5;
rpy_k = str2num(leg.robot.joint{worl_to_base_joint_no}.origin.Attributes.rpy);
R_pj = RPY(rpy_k);
p_pj = str2num(leg.robot.joint{worl_to_base_joint_no}.origin.Attributes.xyz)';
T_wb = [R_pj, p_pj; zeros(1,3), 1];


T_pc = zeros(4,4,4); % transformation between links (parent to child)
T_wk = zeros(4,4,5); T_wk(:,:,1) = T_wb; % transformations in the world frame
for k = 1:4
    % Transformation from parent link frame p to current joint frame
    rpy_k = str2num(leg.robot.joint{k}.origin.Attributes.rpy);
    R_pj = RPY(rpy_k);
    p_pj = str2num(leg.robot.joint{k}.origin.Attributes.xyz)';
    T_pj = [R_pj, p_pj; zeros(1,3), 1]; 
    
    % Tranformation from joint frame of the joint that rotaties body k to
    % link frame. The transformation is pure rotation
    if strcmp(leg.robot.joint{k}.Attributes.type, 'fixed')
        T_jc = eye(4);
    else
        joint_axis_k = str2num(leg.robot.joint{k}.axis.Attributes.xyz)';
        
        R_jk = Rot(q(k), joint_axis_k);
        p_jk = zeros(3,1);
        T_jc = [R_jk, p_jk; zeros(1,3),1];
    end

    % Transformation from parent link frame p to child link frame k
    T_pc(:,:,k) = T_pj*T_jc;

    %  Transformation to base   
    T_wk(:,:,k+1) = T_wk(:,:,k)*T_pc(:,:,k);
end

pos = T_wk(1:3,4,5)


pos_matlab = getTransform(robot, q, robot.BodyNames{5})

jacobian = geometricJacobian(robot, q, robot.BodyNames{5})

% COMPUTE GEOMETRIC JACOBIAN
Jv_wk = zeros(3,3);
Jw_wk = zeros(3,3);
for k = 1:3
    joint_axis_kk = str2num(leg.robot.joint{k}.axis.Attributes.xyz)';
    joint_axis_wk = T_wk(1:3,1:3,k+1)*joint_axis_kk;
    Jv_wk(:,k) = cross(joint_axis_wk, T_wk(1:3,4,5) - T_wk(1:3,4,k+1));
    Jw_wk(:,k) = joint_axis_wk;
end
Jv_wk
Jw_wk








