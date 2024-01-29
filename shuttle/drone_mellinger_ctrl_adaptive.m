% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)
function [T,tau,e_p] = drone_mellinger_ctrl_adaptive(p,v,R,om,P,p_d,psi_d,ie_p,v_d,dpsi_d,a_d,j_d)

% Auxiliary variables
zW = [0;0;1];
zB = R(:,3);
if P.scenario >= 3
  A = P.Pa;

else
  A = [0.6 0 0; 0 0.6 0; 0 0 0.5];
end

% Rotor drag coefficient
dx = 0.50;
dy = 0.236;
dz = 0;
D = diag ([dx, dy, dz]);

J_ref = 0.1;

% Define translation errors
e_p = p - p_d;
e_v = v - v_d;

% Define performance index
J = (e_p'*e_p) + (e_v'*e_v);


% Adaptive gain update
kp_dot = 0.1 * (J - J_ref)' * e_p;
ki_dot = 0.01 * (J - J_ref)' * ie_p;
kv_dot = 0.1 * (J - J_ref)' * e_v;

kp = P.kp + kp_dot;
ki = P.ki + ki_dot;
kv = P.kv + kv_dot;

% Constrain gains
kp = min(max(kp, 15), 20);
ki = min(max(ki, 0), 1);
kv = min(max(kv, 15), 20);

% Control input
J_gain = 0.1; % Adjust this gain to control the sensitivity to the performance index
f_d = -kp*e_p - ki*ie_p - kv*e_v + P.m*P.g*zW + P.m*a_d + J_gain*J*zB;

% Compute thrust
T = f_d'*zB;


if norm(f_d) == 0
    zB_d = [0;0;1];
else
    % Perform division
    zB_d = f_d/norm(f_d);
end

xC_d = [cos(psi_d);sin(psi_d);0];
yB_d = skew(zB_d)*xC_d/norm(skew(zB_d)*xC_d);
xB_d = skew(yB_d)*zB_d;
R_d = [xB_d,yB_d,zB_d];


% compute desired angular velocity
hw_d = P.m/T*(j_d - (zB_d'*j_d)*zB_d);
p_des = -hw_d'*yB_d;
q_des = hw_d'*xB_d;

% Solve for r_des
T_CD=[xC_d, yB_d, zW];
A = R_d'*T_CD;
aux1 = A(1,1)/(A(2,2)*A(1,1)-A(2,1)*A(1,2));
aux2 = q_des - A(2,1)/A(1,1)*p_des + (A(2,1)/A(1,1)*A(1,3) - A(2,3))*dpsi_d;
dtheta_d = aux1*aux2;
dphi_d=(p_des-A(1,2)*dtheta_d-A(1,3)*dpsi_d)/A(1,1);
dlbd_d = [dphi_d;dtheta_d;dpsi_d];
r_des=A(3,:)*dlbd_d;
om_d = [p_des;q_des;r_des];
% Compute torques
e_om = om-om_d;
e_R = 1/2*unskew(R_d'*R - R'*R_d);
tau = -P.kR*e_R - P.kom*e_om;

end
