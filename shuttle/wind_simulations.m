
% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% Summary: Group of basic simulations to know the drone trajectory tracking
% under various speed values

clear all;


% Model and simulation parameters

Param.Tend = 60;
Param.dTi = 0.001;  % inner-loop and simulation sampling period
Nsim = round(Param.Tend/Param.dTi)+1;
Param.g = 9.81;     % earth gravity
Param.nD = 1; % number of drones
Param.scenario=2; %scenario for simulation with melling controller response with uadratic wave


% reference parameters
Param.p_ref_static = [1;1;3];
Param.psi_ref_static = pi/3;

Param.vz_d = 0.1;
Param.dh = 0.05;      % safety height difference between drones
Param.Rad = 5;        % radius of circle

Param.dphase = -pi/12;% ref circle angular difference between drones
Param.ref_mode = 2; % reference: 1 - square wave; 2 - circle
Param.omn = 1;


%air density
Param.air_d = 1.225;
Param.Pa = [0.57 0 0;
    0 0.57 0;
    0 0 0.475];
%Area swept by the rotor
Param.rotor_radius = 0.18;
Param.A = pi*Param.rotor_radius^2;


% M690B drone 
% (guessing parameters! needs identification)
Param.m = 4;        % drone mass (added board)
Param.I = diag([2e-2,2e-2,3e-2]);  % inertia tensor
Param.D = 0.00;     % frame drag coeficient
% Gains for nonlinear controller (crazyflie): OK with dTi = 0.001 (not OK for dTi >0.05)
% Param.kp = diag([10,10,6]);
% Param.kv = diag([5,5,5]);
% Param.ki = zeros(3);
% Param.kR = diag([15,15,15]);
% Param.kom= diag([1,1,1]);
Param.kp = diag([20,20,20]);
Param.kv = diag([10,10,10]);
Param.ki = 0*diag([2,2,2]);
Param.kR = diag([30,30,30]);
Param.kom= diag([1,1,1]);


% initialize variables for all drones:
t = 0:Param.dTi:Param.Tend;
nt = length(t);
nx = 18;
nu = 4;

% simulations parameters
tracking_sim = 0;
situation = 0;
Param.Vw = [0;0;0];
rmse_values_wind = [];
wind_values_x = [[0; 0; 0] ,[-2; 0; 0], [-5; 0; 0], [-10; 0; 0], [-15; 0; 0]];
wind_values_y = [[0; 0; 0] ,[0; 1; 0], [0; 2; 0], [0; 5; 0], [0; 10; 0]];
wind_values_z = [[0; 0; 0] ,[0; 0; 1], [0; 0; 2], [0; 0; 5], [0; 0; 10]];

wind_values = wind_values_x;


Param.scenario=2;
for i = 1:5
 
 Param.Vw = wind_values(:,i);
 trajectory;
 drone_main_simul;
 show_simulations_plots = 0;
 drone_show_data;
 rmse_values_wind = [rmse_values_wind,rmse_value];
end

Param.scenario=3;
for i = 1:5
 
 Param.Vw = wind_values(:,i);
 trajectory;
 drone_main_simul;
 show_simulations_plots = 0;
 drone_show_data;
 rmse_values_wind = [rmse_values_wind, rmse_value];
end

Param.scenario=4;
for i = 1:5
  
 Param.Vw = wind_values(:,i);
 trajectory;
 drone_main_simul;
 show_simulations_plots = 0;
 drone_show_data;
 rmse_values_wind = [rmse_values_wind, rmse_value];
end

Param.scenario=5;
for i = 1:5
 Param.Vw = wind_values(:,i);
 trajectory;
 drone_main_simul;
 show_simulations_plots = 0;
 drone_show_data;
 rmse_values_wind = [rmse_values_wind, rmse_value];

end

% Rearrange the matrix into a 4x4 matrix
rearrangedMatrix = reshape(rmse_values_wind, [], 4);

rmse_values = round(rearrangedMatrix, 3);

fprintf('/n/n/n');

fprintf('(0,0,0) & %f & %f & %f & %f\n',rmse_values(1,1),rmse_values(1,2),rmse_values(1,3),rmse_values(1,4));
fprintf('(2,2,2) & %f & %f & %f & %f\n',rmse_values(2,1),rmse_values(2,2),rmse_values(2,3),rmse_values(2,4));
fprintf('(5,5,5) & %f & %f & %f & %f\n',rmse_values(3,1),rmse_values(3,2),rmse_values(3,3),rmse_values(3,4));
fprintf('(10,0,0) & %f & %f & %f & %f\n',rmse_values(4,1),rmse_values(4,2),rmse_values(4,3),rmse_values(4,4));
fprintf('(15,0,0) & %f & %f & %f & %f\n',rmse_values(5,1),rmse_values(5,2),rmse_values(5,3),rmse_values(5,4));