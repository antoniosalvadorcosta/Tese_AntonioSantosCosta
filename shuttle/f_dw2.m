function Vc = f_dw2(p1,p2, T1, P)


% downwash shapping parameters sugested by Gemini
k = 1.75;                % between 0 and 1
h = 1.15;          % in the range of rotor diameter or slightly larger
disk_area = pi*P.rotor_radius^2;

height_diff = p1(3)-p2(3);

% Drone 1 induced velcoity
vi = sqrt(T1/(2*P.air_d*disk_area));

% Drone 1 downwash velocity
vc = vi + vi*tanh(-k*(height_diff)/h);

% radial distance where downwash becames negligible
dw_radius = sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2);

condition = P.rotor_radius ./(sqrt(1 + tanh(-k*( height_diff )/h)));

if P.use ==1
    
    if  dw_radius < condition
         % downwash vertical velocity
        Vc = vc;
    else
         Vc = 0 ; % Add 0 if condition not met
    end
else
    Vc = vc;
    
end


end