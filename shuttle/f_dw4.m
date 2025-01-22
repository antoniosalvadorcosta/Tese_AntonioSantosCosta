function Vc = f_dw4(p1,p2, T1, P)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
disk_area = pi*P.rotor_radius^2;
height_diff = p1(3)-p2(3);

% Drone 1 induced velcoity
vi = sqrt(T1/(2*P.air_d*disk_area));

% Drone 1 downwash velocity
Vc = (10*vi/height_diff )* sqrt(disk_area/2*pi);
end

