function [dp,dv,dR,dom, vi] = drone_3dfull_dyn_cpte(v,R,om,T,tau,P,iD,dw)
%DRONE_MODEL Summary of this function goes here
%   Detailed explanation goes here

global vi_d2;
global pos_d2;

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


% downwash parameters
k = P.d_k;
h = P.d_h;

% rotor drag coeficient
dx = 0.50;
dy = 0.39;
dz = 0.11;
D = diag ([dx, dy, dz]);

c = T/P.m + 0.09*(v'*(R(:,1)+R(:,2))).^2;

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
    vd = 0;
    if T < 0
        disp('Drone unstable!');
        disp(T);
    else
          vi = sqrt(T/(2*P.air_d*P.A));
        
%         % ID = 1
%         if iD == 1
%             
%             % height differenc4e between rotor and a point bellow
%             height_diff = p(3,:) - pos_d2(3,:);
%             
%             % induced velocity
%             vi = sqrt(T/(2*P.air_d*P.A));
%             
%             % radial distance between rotor center and the downwash
%             % boundary (Momentum theory)
%             dw_radius = sqrt( (T) / (pi * P.air_d  * vi^2) );
%             
%             condition =  P.rotor_radius / sqrt(1 + tanh(-k*( height_diff )/h));
%             
%             if dw_radius < condition
%                 % downwash vertical velocity
%                 vd = vi + vi*tanh(-k*(height_diff )/h);
%             else
%                 vd = 0;
%             end
%             
%             % ID = 2
%         else
%             vi = sqrt(T/(2*P.air_d*P.A));
%             pos_d2 = p;
%         end
    end
    
    if iD == 1
      
        aux = v-P.Vw-R*[0;0;vi];
        v_air = aux(1:3,1);
        
    else
        
        aux = v-P.Vw-R*[0;0;vi]-[0;0;dw];
        v_air = aux(1:3,1);
    end
    
    % rotor drag force
    rotor_drag_force = -R*D*R'*v_air;
    
    % Body drag coeficient and force
    Cd = 0.03 * A / (P.m^0.5); %  (Schneider & Peters, 2001)
    %Cd = 0.0346 * A .* (((aux(1:3,1)).^2)/(P.m^0.6)); % 2022, Analytical and experimental study of quadrotor body drag coefficients considering different quadrotor shapes" by Cai et al. (2022) in the journal Aerospace Science and Technology
    
    %frame_drag = (Cd * A * ((aux(1:3,1)).^2))/2; % He, C., Li, Z., & Xie, H. (2012). Investigation of quadrotor aerodynamic characteristics at different flight speeds
    frame_drag = R*(1/2)*P.air_d*Cd.*A*(R'*v_air).^2;%.*sign(aux(1:3,1));
    
    dp = v;       % T/P.m*zB
    dv = -P.g*zW + T/P.m*zB + rotor_drag_force - frame_drag;
    dR = R*skew(om);
    dom = P.I^(-1)*(-skew(om)*P.I*om + tau);
end

%     if t > 0.001
%         test = 1;
%     end

end

