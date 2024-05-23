function V_dw = f_dw3_0(p1,p2, T1,v1, P)



disk_area = pi*P.rotor_radius^2;

radial_dist = sqrt((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2);

axial_diff = p1(3)- p2(3);

v_air = v1 - P.Vw;


% Drone above induced velocity in hovering
vh = sqrt(T1/(2*P.air_d*disk_area));

vi = 0;
% Drone above induced velocity
vi = (vh^2)/(sqrt(v_air(1)^2 + v_air(2)^2 + (vi + v_air(3))^2));

if all(v1 == [0; 0; 0]) && all(P.Vw == [0; 0; 0])
    vi = vh;
end



% Vmax around centerline
v_max = vi* (P.Cax * P.L/(axial_diff));

% Downwash model of the drone above
V_dw = v_max * exp(-P.Crad * (radial_dist /axial_diff)^2);

end

