function [dp,dv,dR,dom] = drone_3dfull_dyn(v,R,om,T,tau,P)
%DRONE_MODEL Summary of this function goes here
%   Detailed explanation goes here

    % auxiliary variables
    zW = [0;0;1];
    zB = R(:,3);
    
    
    % equations of motion
    if P.scenario <= 2
    dp = v;
    dv = -P.g*zW + T/P.m*zB; %- R*P.D*R'*v;
    dR = R*skew(om);
    dom = P.I^(-1)*(-skew(om)*P.I*om + tau);
    end
    
    
    %---------------------Trajectory following by one drone ---------------
    if P.scenario == 3 || P.scenario == 4
        
    if T < 0
       disp('Drone unstable!');
       disp(T);  
    else
        vi = sqrt(T/(2*P.air_d*P.A));
    end
       
    aux = v-P.Vw-vi;    
    dp = v;
    dv = -P.g*zW + T/P.m*zB - R*P.D*R'*v -(1/2)*P.air_d*P.D*P.Pa*((aux(1:3,1)).^2).*sign(aux(1:3,1));
    dR = R*skew(om);
    dom = P.I^(-1)*(-skew(om)*P.I*om + tau);
    end
    
%     if t > 0.001
%         test = 1;
%     end
    
end

