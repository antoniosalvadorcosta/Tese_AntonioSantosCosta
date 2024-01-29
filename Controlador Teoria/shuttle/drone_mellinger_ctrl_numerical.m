function [T,tau,e_p] = drone_mellinger_ctrl_numerical(p,v,R,om,P,p_d,psi_d,ie_p,v_d,dpsi_d,a_d,j_d)
% auxiliary variables
zW = [0;0;1];
zB = R(:,3);

if P.scenario >= 3
    A = P.Pa;
else
    A = [0.6 0 0; 0 0.6 0; 0 0 0.5];
end
dx = 0.50;
dy = 0.236;
dz = 0;
D = diag ([dx, dy, dz]);

% define translation errors
e_p = p - p_d;
e_v = v - v_d;

obj = [];
constraints = [];


% compute desired force vector with attitude
T = fmincon(@(f_d) cost_function(f_d,p,v,R,om,p_d,psi_d,ie_p,v_d,dpsi_d,a_d,j_d,zW,zB,P), 0.1*zB, obj, constraints);

% compute desired rotation matrix
zB_d = T/norm(T);

xC_d = [cos(psi_d);sin(psi_d);0];
yB_d = skew(zB_d)*xC_d/norm(skew(zB_d)*xC_d);
xB_d = skew(yB_d)*zB_d;
R_d = [xB_d,yB_d,zB_d];

% compute desired angular velocity
hw_d = P.m/T*(j_d - (zB_d'*j_d)*zB_d);
p_des = -hw_d'*yB_d(2);
q_des = hw_d'*xB_d(2);

% define desired angular velocity
% Solve for r_des
T_CD = [xC_d, yB_d, zW];
A = R_d'*T_CD;
aux1 = A(1,1) / (A(2,2) * A(1,1) - A(2,1) * A(1,2));
aux2 = q_des - A(2,1) / A(1,1) * p_des +  (A(2,1) / A(1,1) * A(1,3) - A(2,3)) * dpsi_d;
dtheta_d = aux1 * aux2;
dphi_d = (p_des - A(1,2) * dtheta_d - A(1,3) * dpsi_d) / A(1,1);
dlbd_d = [dphi_d; dtheta_d; dpsi_d];
r_des = A(3,:) * dlbd_d;
om_d = [p_des; q_des; r_des];

% compute torques
e_om = om - om_d;
e_R = 1/2*unskew(R_d'*R - R'*R_d);
tau = -P.kR*e_R - P.kom*e_om;
end