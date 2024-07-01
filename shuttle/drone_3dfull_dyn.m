function [dp,dv,dR,dom, v_air] = drone_3dfull_dyn(v,R,om,T,tau,P)
%DRONE_MODEL Summary of this function goes here
%   Detailed explanation goes here

% auxiliary variables
zW = [0;0;1];
zB = R(:,3);
if P.scenario > 2
    rotor_rad = P.rotor_radius;
    A = P.Pa;
end
A = P.Pa;
vw = P.Vw;
kh = 0.09;

% rotor drag coeficient
dx = 0.50;
dy = 0.39;
dz = 0.11;
D = diag ([dx, dy, dz]);

% equations of motion
if P.scenario <= 1
    dp = v;
    dv = -P.g*zW + T/P.m*zB;
    dR = R*skew(om);
    dom = P.I^(-1)*(-skew(om)*P.I*om + tau);
end


% Take in consideration every aerodynamic effect (Rotor drag and frame
% drag)
if P.scenario > 1
  
    if T < 0
        disp('Drone unstable!');
        disp(T);
    else
        v_air = v - vw;

    end
    
    % rotor drag acceleration
    rotor_drag_a = R*D*R'*v_air;
    
    % Body drag coeficient and acceleration
    Cd = 1.18 ; % Aerodynamics aeronautics and flight mechanics
    
    frame_drag_a = (1/2)*P.air_d*Cd.*A*(v_air.^2).*sign(v_air)/P.m;
    
    dp = v;      
    dv = -P.g*zW + T/P.m*zB - rotor_drag_a - frame_drag_a;
    dR = R*skew(om);
    dom = P.I^(-1)*(-skew(om)*P.I*om + tau);
end

%     if t > 0.001
%         test = 1;
%     end

end

