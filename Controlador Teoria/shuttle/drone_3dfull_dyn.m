function [dp,dv,dR,dom] = drone_3dfull_dyn(v,R,om,T,tau,P)
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
    z_drone_bellow = 0;
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
           warning('Drone unstable!');
           disp(T);  
        else
            % Calculate the induced speed
            vi = sqrt(T/(2*P.air_d*P.A));
            
             if P.scenario > 5
                % Calculate the produced downwash
                vd = vi + vi*((1/4)*tanh(0.18));
                %disp(vd);
             end
            
        end
        
        % calculate relative air speed
        aux = v-vw-R*[0;0;vi];
        v_air = aux(1:3,1);
        
        % rotor drag force
        rotor_drag_force = -R*D*R'*v_air;
         
        % Body drag coeficient and force
        Cd = 0.03 * A / (P.m^0.5); %  (Schneider & Peters, 2001)
        %Cd = 0.0346 * A .* (((aux(1:3,1)).^2)/(P.m^0.6)); % 2022, Analytical and experimental study of quadrotor body drag coefficients considering different quadrotor shapes" by Cai et al. (2022) in the journal Aerospace Science and Technology
       
        %frame_drag = (Cd * A * ((aux(1:3,1)).^2))/2; % He, C., Li, Z., & Xie, H. (2012). Investigation of quadrotor aerodynamic characteristics at different flight speeds
        frame_drag = R*(1/2)*P.air_d*Cd.*A*(R'*v_air.^2);%.*sign(aux(1:3,1));
   
        dp = v;
        dv = -P.g*zW + T/P.m*zB + rotor_drag_force - frame_drag; 
        dR = R*skew(om);
        dom = P.I^(-1)*(-skew(om)*P.I*om + tau);
    end
    
%     if t > 0.001
%         test = 1;
%     end
    
end

