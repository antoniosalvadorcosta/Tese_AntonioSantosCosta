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
Param.psi_ref_static = pi/2;

Param.vz_d = 0.1;
Param.dh = 0.05;      % safety height difference between drones
Param.Rad = 5;        % radius of circle
Param.omn = 0.2;  % rotation frequency
Param.dphase = -pi/12;% ref circle angular difference between drones
Param.Vw = 0;
Param.ref_mode = 1; % reference: 1 - square wave; 2 - circle

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

% Param.kp = diag([20,20,20]);
% Param.kv = diag([10,10,10]);
% Param.ki = diag([2,2,2]);
% Param.kR = diag([30,30,30]);
% Param.kom= diag([1,1,1]);

Param.Vw = [0;0;0];

%air density
Param.air_d = 1.225;
Param.Pa = [0.57 0 0;
    0 0.57 0;
    0 0 0.475];
%Area swept by the rotor
Param.rotor_radius = 0.18;
Param.A = pi*Param.rotor_radius^2;

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
        % Define synchronization time
        t_step = 8;

        % Generate switching signal
        sw1 = mod(t,t_step) >= 5; % position
        sw2 = mod(t,t_step) >= 2; % yaw angle

        % Define position reference
        p_ref{iD} = (p0{iD}*ones(1,Param.Tend/Param.dTi+1) + Param.p_ref_static*sw1)*2;

        % Set velocity, acceleration, and jerk references to zeros
        v_ref{iD} = zeros(3,nt);
        a_ref{iD} = zeros(3,nt);
        j_ref{iD} = zeros(3,nt);

        % Generate attitude reference
        psi_ref{iD} = psi0{iD}*ones(1,Param.Tend/Param.dTi+1) + Param.psi_ref_static*sw2;

        % Set yaw rate reference to zeros
        dpsi_ref{iD} = zeros(1,Param.Tend/Param.dTi+1);
        
        end   

        p_ref_all{iD} = [p_ref{iD};v_ref{iD};a_ref{iD};j_ref{iD}];
        psi_ref_all{iD} = [psi_ref{iD};dpsi_ref{iD}];
    
end
