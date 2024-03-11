%% 2D Simulation

% drones' characteristics
m = 5;
rotor_r = 0.18;
T1 = 60;
disk_area = pi*rotor_r^2;
air_d = 1.225;

% drone's 2 thrust in hover state
T2 = m * 9.81;

% thrust per rotor disk
T1 = T1/4;
T2 = T2/4;

% simulation setup
z_r = 3.0;
z = 2:0.01:z_r;

height_diff = z_r - z;

% downwash shapping parameters
% sugested by Gemini
k = 0.8;                % between 0 and 1
h = 2*rotor_r;          % in the range of rotor diameter or slightly larger

% Drone 1 induced veocity per rotor 
vi1 = sqrt(T1/(2*air_d*disk_area));

% Drone 1 downwash velocity
vc = vi1  + vi1*tanh(-k*(height_diff)/h);

% Drone 2 induced velocity
vi2 = sqrt(T2/(2*air_d*disk_area));

% velocity necessary to affect
v_turb = vi2 * 0.20;

% position in z where downwash becames negligible
z_dw_bound = z_r - (h/k) * atanh((v_turb/vi1) - 1);

% radial distance where downwash becames negligible
dw_radius = z_dw_bound - z_r ;

condition = rotor_r ./sqrt(1 + tanh(-k*( height_diff )./h));

vc(condition > dw_radius) = 0;

figure(7);
plot(z, vc, 'b');
hold off;
grid on;
xlabel('$$z$$ [m]');
legend('Downwash velocity')
ylabel('$$Downwash$$ [m/s]');



%% 3D Vectorial field
% Define the position of the rotor disk
disk_center = [1, 1, 3];
disk_radius = 0.18;

[X, Y, Z] = meshgrid(0:0.10:3, 0:0.10:3,0:0.15:3);
x = reshape(X, [], 1);
y = reshape(Y, [], 1);
z = reshape(Z, [], 1);

% Initialize arrays for velocity components
u = zeros(size(x));
v = zeros(size(y));
w = zeros(size(z));

% Compute the velocity field based on the downwash vector at each point
for i = 1:numel(x)
    % Calculate the radial distance from the rotor disk center
    radial_dist = sqrt((x(i) - disk_center(1))^2 + (y(i) - disk_center(2))^2);
    
    % Check if the point is within the rotor disk
    if radial_dist <= disk_radius
        w(i) = vd_store(i); % Inside the rotor disk
    else
        w(i) = 0; % Outside the rotor disk (no downwash)
    end
end

% Visualize the vector field
quiver3(x, y, z, u, v, w, 'b'); % Blue arrows representing the field
hold on;

% Plot the rotor disk (a circle in 3D space)
theta = linspace(0, 2*pi, 100);
disk_x = disk_center(1) + disk_radius * cos(theta);
disk_y = disk_center(2) + disk_radius * sin(theta);
disk_z = disk_center(3) * ones(size(theta));



plot3(disk_x, disk_y, disk_z, 'g', 'LineWidth', 0.5);


% Define the number of propeller blades
num_blades = 8;

% Create propeller blades (assuming they are evenly spaced)
blade_angles = linspace(0, 2*pi, num_blades + 1);
blade_length = 0.18; % Adjust blade length as needed

for i = 1:num_blades
    blade_start_x = disk_center(1) + 0.001 * disk_radius * cos(blade_angles(i));
    blade_start_y = disk_center(2) + 0.001 * disk_radius * sin(blade_angles(i));
    blade_end_x = blade_start_x + blade_length * cos(blade_angles(i));
    blade_end_y = blade_start_y + blade_length * sin(blade_angles(i));
    
    % Plot each blade
    plot3([blade_start_x, blade_end_x], [blade_start_y, blade_end_y], [disk_center(3), disk_center(3)], 'g', 'LineWidth', 1);
end


xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Vector Field of Downwash Speed Below Rotor Disk');
grid on;
axis tight;
view(30, 30); % Adjust the view angle
hold off;



