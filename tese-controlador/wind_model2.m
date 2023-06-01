function [aux, wind_from_up, produced_downwash]  = wind_model2(Wv,u,v,wc, R,p,Parameter)

P = Parameter;

% Wind speed felt on the body
Vw = R*Wv;

vi_2 = 0;
if u(1) < 0
    disp('Drone 2 unstable!');
    disp(u(1));
    
else
    
    % if V==0
    
    %T = (u(1)/(1-(P.rotor_radius/((4*((p(3)-(P.height+3.00))))))^2));
   
    if u(1) < 0
        disp('Aii!-------------------------------------------------------------------------');
        % sqrt(u(1)/(2*P.air_d*P.A));
        % vi = sqrt(-T/(2*P.air_d*P.A));
    else
        
        vi_2 = sqrt(u(1)/(2*P.air_d*P.A));
    end
end
 
         
% if V==0
 disp('wind from up: ');
 disp(wc);

% downwash felt on the drone  
wind_from_up = R*[0,0,wc]';

produced_downwash = vi_2;

if P.scenario == 2
aux = v - wind_from_up - produced_downwash;
else
   aux = v - Vw - produced_downwash; 
   disp('Vw2:');
   disp(Vw);
end

% Relative wind speed
% V = Vw-v;
 


