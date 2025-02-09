% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% inicializations

clear all;

% Model and simulation parameters
Param.Tend = 60;
Param.dTi = 0.001;  % inner-loop and simulation sampling period
Nsim = round(Param.Tend/Param.dTi)+1;
Param.g = 9.81;     % earth gravity
Param.nD = 1; % number of drones
Param.scenario=4; %scenario for simulation with melling controller considering rotor drag to response with uadratic wave


% reference parameters
Param.p_ref_static = [1;1;3];
Param.psi_ref_static = pi/3;

Param.vz_d = 0.1;
Param.dh = 0.05;      % safety height difference between drones
Param.Rad = 5;        % radius of circle
Param.omn = 0.2;  % rotation frequency
Param.dphase = -pi/12;% ref circle angular difference between drones
Param.ref_mode = 2; % reference: 1 - square wave; 2 - circle
Param.Vw = [0;0;0];

% M690B drone 
% (guessing parameters! needs identification)
Param.m = 5;        % drone mass (added board)
Param.I = diag([2e-2,2e-2,3e-2]);  % inertia tensor
 
Param.kp = diag([20,20,20]);
Param.kv = diag([10,10,10]);
Param.ki = diag([2,2,2]);
Param.kR = diag([30,30,30]);
Param.kom= diag([1,1,1]);



%air density
Param.air_d = 1.225;

%Projected Area
Param.Pa = [0.6 0 0;
            0 0.6 0;
            0 0 0.5];
% Param.width = 0.6;
% Param.length = 0.6;
% Param.height = 0.5;
% Param.Pa = (Param.width * Param.length)/2;

        
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
    
    if Param.ref_mode == 2 % circle reference
        phase{iD} = (iD-1)*Param.dphase;
       
        p_ref{iD} = [Param.Rad*cos(Param.omn*t+phase{iD});Param.Rad*sin(Param.omn*t+phase{iD});Param.vz_d*t+Param.p_ref_static(3)];
        v_ref{iD} = [-Param.Rad*Param.omn*sin(Param.omn*t+phase{iD});Param.Rad*Param.omn*cos(Param.omn*t+phase{iD});Param.vz_d*ones(size(t))];
        a_ref{iD} = [-Param.Rad*Param.omn^2*cos(Param.omn*t+phase{iD});-Param.Rad*Param.omn^2*sin(Param.omn*t+phase{iD});0*ones(size(t))];
        j_ref{iD} = [ Param.Rad*Param.omn^3*sin(Param.omn*t+phase{iD});-Param.Rad*Param.omn^3*cos(Param.omn*t+phase{iD});0*ones(size(t))];
        psi_ref{iD} = atan2(v_ref{iD}(2,:),v_ref{iD}(1,:));
        dpsi_ref{iD} = Param.omn*ones(size(psi_ref{iD}));
       
    end
    if Param.ref_mode == 3 % leniscate reference
        % constants
        a = 0.5; % semi-major axis
        b = 0.2; % semi-minor axis

        % calculate time-dependent position, velocity, acceleration, and angular velocity
        phase = 2*pi*t + iD*Param.dphase;

        p_ref{iD} = [a*cos(phase), a*sin(phase), 0];
        v_ref{iD} = [-a*sin(phase), a*cos(phase), 0];
        a_ref{iD} = [-a^2*cos(2*phase), -a^2*sin(2*phase), 0];
        j_ref{iD} = [-a^3*sin(2*phase), a^3*cos(2*phase), 0];
        psi_ref{iD} = atan2(v_ref{iD}(2), v_ref{iD}(1));
        dpsi_ref{iD} = 2*pi*ones(size(psi_ref{iD}));
    
    end 
    
    p_ref_all{iD} = [p_ref{iD};v_ref{iD};a_ref{iD};j_ref{iD}];
    psi_ref_all{iD} = [psi_ref{iD};dpsi_ref{iD}];
    
end
