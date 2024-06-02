function [dp,dv,dR,dom] = drone_3dfull_dyn_cpte(p,v,R,om,T,tau,F,P,other_p, other_T, other_v)
%DRONE_MODEL Summary of this function goes here
%   Detailed explanation goes here

global v_air_store;

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
        disp('Drone unstable (model)!');
        disp(T);
   
    end
    
%     if T > 100
%         
%         T = 100;
%     end
    
    if other_p(3) > p(3)

        dw =  f_dw3_0(other_p,p,other_T, other_v,P);
        v_air = v- P.Vw - [0;0;dw];
        v_air_store = [v_air_store; v_air(3)];
        % Drone below induced velocity in hovering
        vh = sqrt(T/(2*P.air_d*P.rotor_radius^2*pi));

        vi = 0;
        % Drone below induced velocity
        vi = (vh^2)/(sqrt(v_air(1)^2 + v_air(2)^2 + (vi + v_air(3))^2));
        
      
    else
       
        aux = v-P.Vw;
        v_air = aux(1:3,1);
    end
    
    % rotor drag force
    rotor_drag_a = R*D*R'*v_air;
    
    % Body drag coeficient and force
    
    %Cd = 0.03 * A / (P.m^0.5); %  (Schneider & Peters, 2001)
    Cd = 1.18;
    %Cd = 0.0346 * A * (((v_air).^2)/(P.m^0.6));
                                                
    frame_drag_force = (1/2)*P.air_d*Cd.*A*(v_air.^2).*sign(v_air);
    total_disturb_force =0 ;
   
    dp = v;      
    dv = -P.g*zW + (T/P.m)*zB + rotor_drag_a + frame_drag_force/P.m; 
    dR = R*skew(om);
    dom = P.I^(-1)*(-skew(om)*P.I*om + tau);
end

%     if t > 0.001
%         test = 1;
%     end

end

