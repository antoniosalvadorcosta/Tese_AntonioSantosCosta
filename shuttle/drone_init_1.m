% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% inicializations

clear all;


% Model and simulation parameters
Param.Tend = 8;
Param.dTi = 0.001;  % inner-loop and simulation sampling period
Nsim = round(Param.Tend/Param.dTi)+1;
Param.g = 9.81;     % earth gravity
Param.nD = 1; % number of drones
Param.scenario = 1; %scenario for simulation with melling controller response with uadratic wave


% reference parameters
Param.p_ref_static = [1;1;3];
Param.psi_ref_static = pi/3;

Param.vz_d = 0.1;
Param.dh = 0.05;      % safety height difference between drones
Param.Rad = 5;        % radius of circle
Param.omn = 2*pi/20;  % rotation frequency
Param.dphase = -pi/12;% ref circle angular difference between drones

Param.ref_mode = 1; % reference: 1 - square wave; 2 - circle

% M690B drone 
% (guessing parameters! needs identification)
Param.m = 5;        % drone mass (added board)
Param.I = diag([2e-2,2e-2,3e-2]);  % inertia tensor
Param.D = 0.00;     % frame drag coeficient
% Gains for nonlinear controller (crazyflie): OK with dTi = 0.001 (not OK for dTi >0.05)
% Param.kp = diag([10,10,6]);
% Param.kv = diag([5,5,5]);
% Param.ki = zeros(3);
% Param.kR = diag([15,15,15]);
% Param.kom= diag([1,1,1]);
Param.kp = diag([15,15,15]);
Param.kv = diag([15,15,15]);
Param.ki = diag([0,0,0]);
Param.kR = diag([8,8,8]);
Param.kom= diag([0.5,0.5,0.5]);

% % Crazyflie 2.0 based on 
% % Benoit Landry, "Planning and Control for Quadrotor Flight through Cluttered Environments", BSc, MIT, 2014
% Param.m = 0.031;      % mass (added board)
% Param.I = diag([2.3951e-5,2.3951e-5,3.2347e-5]);  % inertia tensor
% Param.D = 0.001;      % frame drag coeficient
% % Gains for nonlinear controller (crazyflie): OK for dTi = 0.001 -> 0.05
% Param.kp = diag([8,8,12])*1e-2;
% Param.kv = diag([8,8,10])*1e-2;
% Param.ki = diag([0,0,0]);
% Param.kR = diag([20,20,20])*1e-3;
% Param.kom= diag([5,5,5])*1e-3;

% % Iris drone
% % Benoit Landry, "Planning and Control for Quadrotor Flight through Cluttered Environments", BSc, MIT, 2014
% Param.m = 1.5;          % mass (added board)
% Param.I = diag([3.5e-3,3.5e-3,5e-3]);  % inertia tensor
% Param.D = 0.00;       % frame drag coeficient
% % Gains for nonlinear controller (crazyflie): OK with dTi = 0.001 (not OK for dTi >0.05)
% % Param.kp = diag([10,10,6]);
% % Param.kv = diag([5,5,5]);
% % Param.ki = zeros(3);
% % Param.kR = diag([15,15,15]);
% % Param.kom= diag([1,1,1]);
% Param.kp = diag([8,8,6]);
% Param.kv = diag([5,5,5]);
% Param.ki = diag([0,0,0]);
% Param.kR = diag([8,8,8]);
% Param.kom= diag([0.5,0.5,0.5]);

% initialize variables for all drones:
t = 0:Param.dTi:Param.Tend;
nt = length(t);
nx = 18;
nu = 4;
for iD = 1:Param.nD

    % set initial conditions
    p0{iD} = [0;0.2*((Param.nD-1)/2-iD+1);0];
    v0{iD} = [0;0;0];
    psi0{iD} = pi/20*((Param.nD-1)/2-iD+1);
    R0{iD} = Euler2R([0;0;psi0{iD}]);
    om0{iD} = [0;0;0];
    x{iD} = zeros(nx,Nsim+1);
    xiep{iD} = zeros(3,Nsim+1);
    T{iD} = zeros(1,Nsim);
    tau{iD} = zeros(3,Nsim);
    lbd{iD} = zeros(3,Nsim);
    x{iD}(:,1) = [p0{iD};v0{iD};reshape(R0{iD},[],1);om0{iD}];
    
    if Param.ref_mode == 1 % square wave reference
        t_step = 8;
        %sw = mod(t,t_step)>=t_step/2; % boolean wave
        sw = mod(t,t_step)>=2;
        p_ref{iD} = (p0{iD}*ones(1,Param.Tend/Param.dTi+1) + Param.p_ref_static*sw)*5;
        v_ref{iD} = zeros(3,nt);
        a_ref{iD} = zeros(3,nt);
        j_ref{iD} = zeros(3,nt);
        psi_ref{iD} = psi0{iD}*ones(1,Param.Tend/Param.dTi+1) + Param.psi_ref_static*sw;
        dpsi_ref{iD} = zeros(1,Param.Tend/Param.dTi+1);
    end   
    
    p_ref_all{iD} = [p_ref{iD};v_ref{iD};a_ref{iD};j_ref{iD}];
    psi_ref_all{iD} = [psi_ref{iD};dpsi_ref{iD}];
    
end
