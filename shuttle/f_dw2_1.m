function [Vc,vi] = f_dw2_1(p1,p2, T1, v_air, P)


% downwash shapping parameters sugested by Gemini
k = P.k;                % between 0 and 1
h = P.h;          % in the range of rotor diameter or slightly larger
disk_area = pi*P.rotor_radius^2;

height_diff = p1(3)-p2(3);


% Drone 1 induced velocity in hovering
vh = sqrt(T1/(2*P.air_d*disk_area));

vi = 0;
% Drone 1 induced velocity
vi = (vh^2)/(sqrt(v_air(1)^2 + v_air(2)^2 + (vi + v_air(3))^2));

vi = vh;
% Drone 1 downwash velocity
vc = vi  + vi*tanh(-k*(height_diff)/h);

% radial distance where downwash becames negligible
dw_radius = sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2);

condition = P.rotor_radius ./(sqrt(1 + tanh(-k*( height_diff )/h)));

if  dw_radius < condition
    % downwash vertical velocity
    Vc = vc;
else
    Vc = 0 ; % Add 0 if condition not met
end



end