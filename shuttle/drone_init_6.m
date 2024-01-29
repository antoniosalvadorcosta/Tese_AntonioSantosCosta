% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% inicializations

clear all;


% Model and simulation parameters
Param.Tend = 60;
Param.dTi = 0.001;  % inner-loop and simulation sampling period
Nsim = round(Param.Tend/Param.dTi)+1;
Param.g = 9.81;     % earth gravity
Param.nD = 2; % number of drones
Param.scenario=6; %scenario for simulation with melling controller considering rotor drag to response with uadratic wave


% reference parameters
Param.p_ref_static = [0;0;0];
Param.psi_ref_static = pi/3;

Param.vz_d = 0.1;
Param.dh = 0.05;      % safety height difference between drones
Param.Rad = 5;        % radius of circle
Param.omn = 0.2;  % rotation frequency
Param.dphase = -pi/12;% ref circle angular difference between drones
Param.ref_mode = 2; % reference: 1 - square wave; 2 - circle
Param.Vw = 1;

% M690B drone 
% (guessing parameters! needs identification)
Param.m = 5;        % drone mass (added board)
Param.I = diag([2e-2,2e-2,3e-2]);  % inertia tensor
Param.D = 0;
Param.kp = diag([20,20,20]);
Param.kv = diag([10,10,10]);
Param.ki = diag([2,2,2]);
Param.kR = diag([30,30,30]);
Param.kom= diag([1,1,1]);
% Param.kp = diag([10,10,6]);
% Param.kv = diag([5,5,5]);
% Param.ki = diag([0.1,0.1,0.1]);
% Param.kR = diag([20,20,20]);
% Param.kom= diag([0.2,0.2,0.2]);


%air density
Param.air_d = 1.225;

%Projected Area

Param.Pa = [0.6 0 0;
            0 0.6 0;
            0 0 0.5];
% Param.width = 0.6;
% Param.length = 0.6;
% Param.height = 0.3;
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
    
    if iD == 1
        p0{iD} = [0;0;0];
    end
    if iD == 2
        p0{iD} = [0;0;5.5];
    end
    
    % set initial conditions
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
    
    if iD == 1 % linear for drone 1
       
        
        p_ref{iD} = [t;3*ones(size(t));3*ones(size(t))];
        v_ref{iD} = [ones(size(t));zeros(size(t));zeros(size(t))];
        a_ref{iD} = [zeros(size(t));zeros(size(t));zeros(size(t))];
        j_ref{iD} = [zeros(size(t));zeros(size(t));zeros(size(t))];
        psi_ref{iD} = atan2(v_ref{iD}(2,:),v_ref{iD}(1,:));
        dpsi_ref{iD} = 0*Param.omn*ones(size(psi_ref{iD}));
       
    end
    
    if iD == 2 % parabolic trajectory
        
        
        p_ref{iD} = [t;3*ones(size(t));7/450*t.^2 + 7/2*t - 787/20];
        v_ref{iD} = [ones(size(t));zeros(size(t)); (0.5 * 2)*(t - 15).* exp((0.5 * (t - 15).^2)/5^2) / 5^2];
        a_ref{iD} = [zeros(size(t));zeros(size(t)); (0.5 * 2)^2 * (t - 15).* exp((0.5 * (t - 15).^2)/5^2)/5^2];
        j_ref{iD} = [zeros(size(t));zeros(size(t)); 2 * (t - 15).* exp((0.5 * (t - 15).^2)/5^2)];
        psi_ref{iD} = atan2(v_ref{iD}(2,:),v_ref{iD}(1,:));
        dpsi_ref{iD} = 0*Param.omn*ones(size(psi_ref{iD}));
       
    end
   
    p_ref_all{iD} = [p_ref{iD};v_ref{iD};a_ref{iD};j_ref{iD}];
    psi_ref_all{iD} = [psi_ref{iD};dpsi_ref{iD}];
    
end
