function V_dw = f_dw3_0(p1,p2, T1, P)

z_0 = 0.33;

disk_area = pi*P.rotor_radius^2;

radial_dist = sqrt((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2);

% distance between drones
z = p1(3)- p2(3);

axial_diff = z - z_0;



% Drone above induced velocity in hovering
vh = sqrt(T1/(2*P.air_d*disk_area));

vi = vh;

% Vmax around centerline
v_max = vi* (P.Cax * P.L/(axial_diff));

% Downwash model of the drone above
V_dw = v_max * exp(-P.Crad * (radial_dist /axial_diff)^2);

end

