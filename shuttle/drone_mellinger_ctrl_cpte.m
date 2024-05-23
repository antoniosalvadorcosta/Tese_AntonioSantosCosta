% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)
function [T,tau,e_p] = drone_mellinger_ctrl_cpte(p,v,R,om,P,p_d,psi_d,ie_p,v_d,dpsi_d,a_d,j_d,dw_on, p_above, T_above, v_above)

%     if ~exist('dpsi_d','var') || isempty(dpsi_d), dpsi_ref = 0; end
%     if ~exist('ie_p','var') || isempty(ie_p), ie_p = zeros(3,1); end
%     if ~exist('v_d','var') || isempty(v_d), v_d = zeros(3,1); end
%     if ~exist('a_d','var') || isempty(a_d), a_d = zeros(3,1); end
%     if ~exist('j_d','var') || isempty(j_d), j_d = zeros(3,1); end

global vd_store;



% auxiliary variables
zW = [0;0;1];
zB = R(:,3);

if P.scenario > 2
    rotor_rad = P.rotor_radius;
    A = P.Pa;
end

% rotor drag coeficient
dx = 0.50;
dy = 0.39;
dz = 0.11;
D = diag ([dx, dy, dz]);

% define translation errors
e_p = p - p_d;
e_v = v - v_d;

% desired force vector with attitude

T = 0;
vi = 0;

b = 3.7*10^-7;
dw = 0;

 % if the compensation for the downwash is activated
if dw_on == 1
    
    % if the other drone at higher altitude
    if p_above(3) > p(3)
        
        
        dw =  f_dw3_0(p_above,p,T_above,v_above,P);
        v_air_drone_below = v - P.Vw - [0;0;dw];
   
        vd_store = [vd_store; dw];

    else
       aux = v-P.Vw;
       v_air_drone_below = aux(1:3,1); 
      
      
    
    end

% no compensation for the downwash
else
    % the other drone is above, still store the downwash values
    if p_above(3) > p(3)
        
        dw =  f_dw3_0(p_above,p,T_above,v_above,P);
        vd_store = [vd_store; dw];
       
    end
    
    aux = v-P.Vw;
    v_air_drone_below = aux(1:3,1);
  
end


v_air = v_air_drone_below;

rotor_drag_force = R*D*R'*v_air;

Cd = 0.03 * A / (P.m^0.5); %  (Schneider & Peters, 2001)
%Cd = 0.0346 * A * (((aux(1:3,1)).^2)/(P.m^0.6)); % 2022, Analytical and experimental study of quadrotor body drag coefficients considering different quadrotor shapes" by Cai et al. (2022) in the journal Aerospace Science and Technology

frame_drag = (1/2)*P.air_d*Cd.*A*(v_air.^2).*sign(v_air);
%frame_drag = (Cd * A * ((aux(1:3,1)).^2))/2; % He, C., Li, Z., & Xie, H. (2012). Investigation

f_dr = -P.kp*e_p - P.ki*ie_p - P.kv*e_v + P.m*P.g*zW + P.m*a_d - rotor_drag_force - frame_drag ; %(1/2)*P.air_d*P.D*A*((aux(1:3,1)).^2).*sign(aux(1:3,1));


% compute thrust
if P.scenario >= 4 
    T= f_dr'*zB; % kh*(v'*(R(:,1)+R(:,2)))^2;
else
    T = f_d'*zB;
end

% compute desired rotation matrix
if P.scenario >= 4
    zB_d = f_dr/norm(f_dr);
else
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

if P.scenario > 3
    dlbd_d = [dphi_d(1);dtheta_d(1);dpsi_d(1)];
else
    dlbd_d = [dphi_d;dtheta_d;dpsi_d];
end

r_des=A(3,:)*dlbd_d;

if P.scenario > 3
    om_d = [p_des(1);q_des(1);r_des(1)];
else
    om_d = [p_des;q_des;r_des];
end

% compute torques
e_om = om-om_d;
e_R = 1/2*unskew(R_d'*R - R'*R_d);
tau = -P.kR*e_R - P.kom*e_om;


%     if t > 0.001
%         fprintf('t = %f: T = %f; tau = [%f %f %f]; e_p = [%f %f %f]; e_v = [%f %f %f]; f_d = [%f %f %f]; e_R = [%f %f %f]; e_om = [%f %f %f].\n',...
%             t,T,tau(1),tau(2),tau(3),e_p(1),e_p(2),e_p(3),e_v(1),e_v(2),e_v(3),...
%             f_d(1),f_d(2),f_d(3),e_R(1),e_R(2),e_R(3),e_om(1),e_om(2),e_om(3));
%         test = 1;
%     end

end



