function Vc = f_dw2(p1,p2, T1, T2, P)


% downwash shapping parameters sugested by Gemini
k = 0.8;                % between 0 and 1
h = 2*P.rotor_radius;          % in the range of rotor diameter or slightly larger
disk_area = pi*P.rotor_radius^2;

height_diff = p1(3)-p2(3);


% Drone 1 induced velcoity
vi1 = sqrt(T1/(2*P.air_d*disk_area));

% Drone 1 downwash velocity
vc = vi1  + vi1*tanh(-k*(height_diff)/h);

% Drone 2 induced velocity
vi2 = sqrt(T2/(2*P.air_d*disk_area));

% velocity necessary to affect drone 2 (20 % of it's induced speed)
v_turb = vi2 * 0.20;

% position in z where downwash becames negligible
z_dw_bound = p1(3) - (h/k) * atanh((v_turb/vi1) - 1);

% radial distance where downwash becames negligible
dw_radius = z_dw_bound - p1(3) ;

condition = P.rotor_radius ./sqrt(1 + tanh(-k*( height_diff )/h));

if  dw_radius > height_diff
    % downwash vertical velocity
    Vc = vc;
else
  
    Vc = 0 ; % Add 0 if condition not met
end



end