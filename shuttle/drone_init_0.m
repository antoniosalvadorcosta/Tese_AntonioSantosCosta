% hovering drone initializations

clear all;

% Model and simulation parameters
Param.Tend = 10;
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
Param.m = 3.55;        % drone mass (added board)
Param.I = diag([2e-2,2e-2,3e-2]);  % inertia tensor
 
Param.kp = diag([15,15,10]);
Param.kv = diag([10,10,10]);
Param.ki = diag([0.01,0.01,0.01]);
Param.kR = diag([15,15,15]);
Param.kom= diag([1,1,1]);

Param.Vw = [0;0;0];

% rotor drag coeficient
dx = 0.50;
dy = 0.39;
dz = 0.11;
Param.D = diag ([dx, dy, dz]);

%air density
Param.air_d = 1.225;
Param.Pa = [0.575 0 0;
    0 0.575 0;
    0 0 0.38];
%Area swept by the rotor
Param.rotor_radius = 0.23;
Param.A = pi*Param.rotor_radius^2;

% downwash
Param.arm_length = 0.35;
Param.L = 2 * Param.arm_length;
Param.Cax = 0.15;
Param.Crad = 0.25;
Param.z_0 = 0.330;

% initialize variables for all drones:
t = 0:Param.dTi:Param.Tend;
nt = length(t);
nx = 18;
nu = 4;
for iD = 1:Param.nD

    % set initial conditions
    p0{iD} = [3;3;3];
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
    
    phase{iD} = (iD-1)*Param.dphase;
     
        
        p_ref{iD} = [3*ones(size(t));3*ones(size(t));3*ones(size(t))];
        v_ref{iD} = [zeros(size(t));zeros(size(t));zeros(size(t))];
        a_ref{iD} = [zeros(size(t));zeros(size(t));  zeros(size(t))];
        j_ref{iD} = [zeros(size(t));zeros(size(t));zeros(size(t))];
        psi_ref{iD} = atan2(v_ref{iD}(2,:),v_ref{iD}(1,:));
        dpsi_ref{iD} = 0*Param.omn*ones(size(psi_ref{iD}));
       
        p_ref_all{iD} = [p_ref{iD};v_ref{iD};a_ref{iD};j_ref{iD}];
        psi_ref_all{iD} = [psi_ref{iD};dpsi_ref{iD}];
    
end