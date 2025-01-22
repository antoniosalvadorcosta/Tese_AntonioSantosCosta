function [dp,dv,dR,dom] = drone_3dfull_dyn_cpte(p,v,R,om,T,tau,P,other_p, other_T)
%DRONE_MODEL Summary of this function goes here
%   Detailed explanation goes here


global v_air_store_2;
% auxiliary variables
zW = [0;0;1];
zB = R(:,3);
if P.scenario > 2
    rotor_rad = P.rotor_radius;
    A = P.Pa;
end
A = P.Pa;
z_drone_bellow = 0;
vw = P.Vw;
kh = 0.09;
b = 3.7 * 10 ^ - 7;


% rotor drag coeficient
 
D = P.D;

if T < 0
    disp('Drone unstable (model)!');
    disp(T);
    
end

%     if T > 100
%
%         T = 100;
%     end

if other_p(3) > p(3)
    
    dw = - f_dw2(other_p,p,other_T, P);
    v_air = v - P.Vw - [0;0;dw];
        
    v_air_store_2 = [v_air_store_2, v_air];
else
    
    aux = v + P.Vw;
    v_air = aux(1:3,1);
    
   
end

% rotor drag force acceleration
rotor_drag_a = R*D*R'*v_air;

% Body drag coeficient and acceleration
Cd = 1.18;
frame_drag_a = (1/2)*P.air_d*Cd.*A*(v_air.^2).*sign(v_air);


dp = v;
dv = -P.g*zW + (T/P.m)*zB - rotor_drag_a - frame_drag_a/P.m;
dR = R*skew(om);
dom = P.I^(-1)*(-skew(om)*P.I*om + tau);


%     if t > 0.001
%         test = 1;
%     end

end

