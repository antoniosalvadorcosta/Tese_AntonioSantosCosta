 function V_dw = f_dw3(p1,p2, T1, P)


% downwash shapping parameters sugested by Gemini
Cax = 0.2;                
Crad = 0.5;    
disk_area = pi*P.rotor_radius^2;

radial_dist = sqrt((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2);

axial_diff = p1(3)- p2(3);

% Drone 1 induced velocity
vi1 = sqrt(T1/(2*P.air_d*disk_area));

% Vmax
v_max = vi1* (Cax * P.L/(axial_diff));

% Drone 1 velocity field
V_dw = v_max * exp(-Crad * (radial_dist /axial_diff)^2);

end