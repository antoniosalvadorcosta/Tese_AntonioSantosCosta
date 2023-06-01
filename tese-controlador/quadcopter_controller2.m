function [u,ep] = quadcopter_controller2(m, p,p_ref,v_ref,a_ref, a_ref_dot, Parameter,v,a,R,W)
%QUADROTOR_CONTROLLER Summary of this function goes here

P = Parameter;
%Assumptions
zW = [0 0 1]';
dx = 0.50;
dy = 0.236;
dz = 0;
D = diag ([dx, dy, dz]);
kh = 0.009;

% alpha = a + P.g*zW + dx*v;
% beta = a + P.g*zW + dy*v;
yC = [-sin(P.psi_dest) cos(P.psi_dest) 0]';
% xb = cross(yC,alpha)/norm(cross(yC,alpha));
% yb = cross(beta,xb)/norm(cross(beta,xb));
% zb = cross(xb,yb);
%
% R = eye(3);
% R(:,1) = xb;
% R(:,2) = yb;
% R(:,3) = zb;

%Errors
ep = p-p_ref;
ev = v-v_ref;

zB = R*zW;

a_fb =  -(P.kp/P.m)*ep - (P.kv/P.m)*ev;
a_rd =  R*D*R'*v_ref; %tinha um "-" no início
a_des = a_fb + a_ref - a_rd + P.g*zW;

%if tt_ref == Tend

if P.wind_model_control == 1
a_des = a_fb + a_ref - a_rd + P.g*zW + (1/2)*P.air_d*P.Cd*P.Pa*(v.^2).*sign(v);
end
     
if  P.rd_model == 1 && P.rd_model_control == 0
    disp('considering rotor drag only on the model');
end
   
%end
if P.rd_model_control == 0
    disp('no rotor drag');
    
    
    F_des = -P.kp*ep - P.kv*ev + P.m*P.g*zW + P.m*a_ref;
    
    % Thrust command
    u1 = F_des'*zB;
    %disp(u1);
    
    
    zB_des = F_des/norm(F_des);
    
    
    
    xC_des = [cos(P.psi_dest) sin(P.psi_dest) 0]';
    yB_des = cross(zB_des,xC_des)/norm(cross(zB_des,xC_des));
    xB_des = cross(yB_des,zB_des);
    
    
    
    WRb_des = [xB_des,yB_des,zB_des];
    
    
    %lbd_full(:,k) = R2Euler(R);
    
    
    %rotation error
    er = (1/2)*inv_skew((WRb_des'*R)-(R'*WRb_des)) ;
    
    
    hWdes= (P.m/u1)*(a_ref_dot -(zB_des'*a_ref_dot)*zB_des);
    wBx_des = -hWdes' * yB_des;
    wBy_des = hWdes' * xB_des;
    wBz_des = (P.dpsi_dest*zW)'*zB_des;
    
    %Desired body rates
    WBdes = [wBx_des wBy_des wBz_des]';
    
    
    %angular velocity error
    ew = W - WBdes;
    
    %remaining control inputs
    ut = -P.kr * er - P.kw * ew;
    
    
    
    u = [u1; ut];
    
else
    
    disp('considering rotor drag on the model and on the controller');
    
    zB_des = a_des/norm(a_des);
    
    
    xC_des = [cos(P.psi_dest) sin(P.psi_dest) 0]';
    yB_des = cross(zB_des,xC_des)/norm(cross(zB_des,xC_des));
    xB_des = cross(yB_des,zB_des);
    
    % %Collective thrust command complete
    u1 = (a_des'*zB - P.kh*(v'*(R(:,1)+R(:,2)))^2)*P.m;
%     u1 = (a_des'*zB ) *P.m;
%     disp('sem a turbulência');
    
    WRb_des = [xB_des,yB_des,zB_des];
    
    
    %lbd_full(:,k) = R2Euler(R);
    
    
    %rotation error
    er = (1/2)*inv_skew((WRb_des'*R)-(R'*WRb_des)) ;
    
    
    hWdes= (P.m/u1)*(a_ref_dot -(zB_des'*a_ref_dot)*zB_des);
    wBx_des = -hWdes' * yB_des;
    wBy_des = hWdes' * xB_des;
    wBz_des = (P.dpsi_dest*zW)'*zB_des;
    
    %Desired body rates
    WBdes = [wBx_des wBy_des wBz_des]';
    
    
    %angular velocity error
    ew = W - WBdes;
    
    %remaining control inputs
    ut = -P.kr * er - P.kw * ew;
    
    
    
    u = [u1; ut];
    
    
end
end