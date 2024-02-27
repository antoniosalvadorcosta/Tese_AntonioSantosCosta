function Vc = f_dw3(p1,p2, T1, T2, P)


% downwash shapping parameters sugested by Gemini
Cax = 0.8;                
Crad = 0.5;    
disk_area = pi*P.rotor_radius^2;

radial_dist = sqrt((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2);
axial_diff = p2(3)- p1(3);


% Drone 1 induced velcoity
vi1 = sqrt(T1/(2*P.air_d*disk_area)) * (Cax * 0.6/(axial_diff));

% Drone 2 velocity field

Vc = vi1 * exp(-Crad * (radial_dist /axial_diff));



end