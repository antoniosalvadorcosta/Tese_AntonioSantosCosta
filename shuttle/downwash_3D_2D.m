%% 2D Simulation
imgs_folder = 'simulacoes/capt_man/cpt_';
saves_folder = 'saves/';
if ~exist('filename','var') || isempty(filename)
    %filename = [datestr(now,30) '_simul_'];
    filename = ['_simul_'];
end
do_print = 1; % set to 1 to plot each plot in a PDF file (for latex reports)
do_save_workspace = 0; % set to 1 to save workspace to *.mat file

% downwash shapping parameters sugested by Gemini
% Param.k = 0.8;                % between 0 and 1
% Param.h = 2*P.rotor_radius;          % in the range of rotor diameter or slightly larger


Param.Vw = [0;0;0];
% drones' characteristics

m = Param.m;
rotor_r = Param.rotor_radius;
Total_T = m * 9.81;

T1 = Total_T / 4;
disk_area = pi*rotor_r^2;
arm_length = Param.arm_length;
Param.z_0 = Param.z_0; 
v1 = [0;0;0];
air_d = 1.225;


% simulation setup
z_r = 2.5;
x_r = 2.5;
y_r = 2.5;

z = 0.5:0.01:z_r;
x = 0.5:0.01:x_r;
y = 0.5:0.01:y_r;

height_diff = z_r - z;

% Downwash shapping parameters

Vc = [];

p1 = [x_r;y_r;z_r];
p2 = [x;y;z];
Param.use = 0;
arrived = 0;
% Define the minimum distance for smooth transition
%min_distance =  Param.z_0 + 0.025; % between one or two rotor radius -> steady state
min_distance = 0.025;
Param.axial_test =1;
for i = 1:numel(x)
    aux  = height_diff(i);
    
    if aux > min_distance
       
        vc = f_dw2(p1,p2(:,i),T1, Param); 
    else
        if abs(aux - min_distance) < 1e-1 && arrived == 0
            arrived = 1;
            vc = f_dw2(p1,p2(:,i), T1, Param);
            Vc_zenit = vc;
        end
        if aux < min_distance
        
            vc = 0;
        end
    end 
    Vc = [Vc;vc]; 
end

% Find the index where z exceeds z_r - min_distance
trim_index = find(z >= z_r - min_distance-0.010, 1);

% Trim the vectors z and Vc
z_trimmed = z(1:trim_index);
Vc_trimmed = Vc(1:trim_index);

figure(7);
plot(z_trimmed, Vc_trimmed, 'b');
hold off;
%grid on;
xlabel('$$z$$ [m]');
title('Downwash velocity under rotor disk');
legend('Downwash velocity')
ylabel('$$Downwash$$ [m/s]');
print2pdf([imgs_folder filename '_2D'],do_print);


%% 3D Vectorial field
z_r = 2.5;
x_r = 2.5;
y_r = 2.5;

Param.axial_test = 0;
Param.use = 1;

p1 = [x_r;y_r;z_r];

%Define the position of the rotor disk
disk_center = [x_r, y_r, z_r];
disk_radius = 0.23;

% Create a grid of points in 3D space
x_vals = x_r - 1:0.1:x_r + 1;
y_vals = y_r - 1:0.1:y_r + 1;
z_vals = z_r - 1:0.1:z_r;
[X, Y, Z] = meshgrid(x_vals, y_vals, z_vals);

% Initialize arrays for velocity components
u = zeros(size(X));
v = zeros(size(Y));
w = zeros(size(Z));

% Compute the velocity field based on the downwash vector at each point
for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        for k = 1:size(X, 3)
                      
            w(i, j, k) = f_dw2(p1, [X(i, j, k); Y(i, j, k); Z(i, j, k)], T1, Param);
            
        end
    end
end

% Eliminate negligible values of downwash
nv = 0.01;
w(w<nv) = 0;

% Visualize the vector field
figure(8);
quiver3(X, Y, Z, zeros(size(u)), zeros(size(v)), -w, 'b');
hold on;

z_r = 2.5;
x_r = 2.5;
y_r = 2;


p1 = [x_r;y_r;z_r];

% Define the position of the rotor disk
disk_center = [x_r, y_r, z_r];
disk_radius = 0.23;

% Create a grid of points in 3D space
x_vals = x_r - 1:0.1:x_r + 1;
y_vals = y_r - 1:0.1:y_r + 1;
z_vals = z_r - 1:0.1:z_r;
[X, Y, Z] = meshgrid(x_vals, y_vals, z_vals);

% Initialize arrays for velocity components
u = zeros(size(X));
v = zeros(size(Y));
w = zeros(size(Z));

% Compute the velocity field based on the downwash vector at each point
for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        for k = 1:size(X, 3)
            
                w(i, j, k) = f_dw2(p1, [X(i, j, k); Y(i, j, k); Z(i, j, k)], T1, Param);
            
        end
    end
end

% eliminate negligible values of downwash
 w(w<nv) = 0;

quiver3(X, Y, Z, zeros(size(u)), zeros(size(v)), -w, 'b');
hold on;

z_r = 2.5;
x_r = 2;
y_r = 2;

p1 = [x_r;y_r;z_r];

% Define the position of the rotor disk
disk_center = [x_r, y_r, z_r];
disk_radius = 0.23;

% Create a grid of points in 3D space
x_vals = x_r - 1:0.1:x_r + 1;
y_vals = y_r - 1:0.1:y_r + 1;
z_vals = z_r - 1:0.1:z_r;
[X, Y, Z] = meshgrid(x_vals, y_vals, z_vals);

% Initialize arrays for velocity components
u = zeros(size(X));
v = zeros(size(Y));
w = zeros(size(Z));

% Compute the velocity field based on the downwash vector at each point
for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        for k = 1:size(X, 3)
           
                w(i, j, k) = f_dw2(p1, [X(i, j, k); Y(i, j, k); Z(i, j, k)], T1,  Param);
          
        end
    end
end

% eliminate negligible values of downwash
 w(w<nv) = 0;

quiver3(X, Y, Z, zeros(size(u)), zeros(size(v)), -w, 'b');
hold on;

z_r = 2.5;
x_r = 2;
y_r = 2.5;


p1 = [x_r;y_r;z_r];

% Define the position of the rotor disk
disk_center = [x_r, y_r, z_r];
disk_radius = 0.23;

% Create a grid of points in 3D space
x_vals = x_r - 1:0.1:x_r + 1;
y_vals = y_r - 1:0.1:y_r + 1;
z_vals = z_r - 1:0.1:z_r;
[X, Y, Z] = meshgrid(x_vals, y_vals, z_vals);

% Initialize arrays for velocity components
u = zeros(size(X));
v = zeros(size(Y));
w = zeros(size(Z));

% Compute the velocity field based on the downwash vector at each point
for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        for k = 1:size(X, 3)
            
                w(i, j, k) = f_dw2(p1, [X(i, j, k); Y(i, j, k); Z(i, j, k)], T1,  Param);
            
        end
    end
end

 w(w<nv) = 0;

quiver3(X, Y, Z, zeros(size(u)), zeros(size(v)), -w, 'b');
hold on;

% % % Plot the rotor disk (a circle in 3D space)
% % theta = linspace(0, 2*pi, 100);
% % disk_x = disk_center(1) + disk_radius * cos(theta);
% % disk_y = disk_center(2) + disk_radius * sin(theta);
% % disk_z = disk_center(3) * ones(size(theta));
% % plot3(disk_x, disk_y, disk_z, 'g', 'LineWidth', 2);
% % 
% 
% % Define the number of propeller blades
% num_blades = 8;
% 
% % Create propeller blades (assuming they are evenly spaced)
% blade_angles = linspace(0, 2*pi, num_blades + 1);
% blade_length = 0.23; % Adjust blade length as needed

% for i = 1:num_blades
%     blade_start_x = disk_center(1) + 0.001 * disk_radius * cos(blade_angles(i));
%     blade_start_y = disk_center(2) + 0.001 * disk_radius * sin(blade_angles(i));
%     blade_end_x = blade_start_x + blade_length * cos(blade_angles(i));
%     blade_end_y = blade_start_y + blade_length * sin(blade_angles(i));
%     
%     % Plot each blade
%     plot3([blade_start_x, blade_end_x], [blade_start_y, blade_end_y], [disk_center(3), disk_center(3)], 'g', 'LineWidth', 1);
% end



% Plot the drone model
drone_plot([2.25;2.25;2.5], zeros(3,1), [],'r')

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('Donwash');
title('3D vector field of downwash speed below the shuttle drone');
grid on;
axis tight;
view(30, 30); % Adjust the view angle
hold off;

print2pdf([imgs_folder filename '_3D'],do_print);

