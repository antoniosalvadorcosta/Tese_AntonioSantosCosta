


clear all;

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
Param.Vw = [0;0;0];

%air density
Param.air_d = 1.225;
Param.Pa = [0.575 0 0;
    0 0.575 0;
    0 0 0.38];
%Area swept by the rotor
Param.rotor_radius = 0.23;
Param.A = pi*Param.rotor_radius^2;


% M690B drone 
% (guessing parameters! needs identification)
Param.m = 3.55;        % drone mass (added board)
Param.I = diag([2e-2,2e-2,3e-2]);  % inertia tensor
Param.D = 0.00;     % frame drag coeficient
Param.kp = diag([15,15,10]);
Param.kv = diag([10,10,10]);
Param.ki = diag([0.01,0.01,0.01]);
Param.kR = diag([15,15,15]);
Param.kom= diag([1,1,1]);

% downwash
Param.arm_lenght = 0.33;
Param.L = 0.74;
Param.Cax = 0.1;
Param.Crad = 1.5;


% initialize variables for all drones:
t = 0:Param.dTi:Param.Tend;
nt = length(t);
nx = 18;
nu = 4;


% run simulations
% drone_init_1;
% simple_simulation;
% drone_init_2;
% simple_simulation;
% traj_tracking_simulations;
% wind_simulations;
% lemniscate_4_drones;
% downwash_3D_2D;
Param.ki = 0*diag([0.01,0.01,0.01]);
capt_man_simulations;