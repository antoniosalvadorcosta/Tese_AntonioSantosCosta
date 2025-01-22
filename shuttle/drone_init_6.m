% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% initializations capture maneouvre
clf;

% Model and simulation parameters
Param.Tend = 60;
Param.dTi = 0.001;  % inner-loop and simulation sampling period
Param.Nsim = round(Param.Tend/Param.dTi)+1;
Param.g = 9.81;     % earth gravity
Param.nD = 2; % number of drones
Param.scenario=6; %scenario for simulation with melling controller considering rotor drag to response with uadratic wave
Param.alpha = pi/4;

% reference parameters
Param.p_ref_static = [0;0;0];
Param.psi_ref_static = pi/3;

Param.vz_d = 0.1;
Param.dh = 0.00;      % safety height difference between drones
Param.Rad = 5;        % radius of circle
Param.omn = 0.2;  % rotation frequency
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


% is not a 2D plot of the axial distance
Param.axial_test = 0;


% M690B drone 
% (guessing parameters! needs identification)
Param.m = 3.55;        % drone mass (added board)
Param.I = diag([2e-2,2e-2,3e-2]);  % inertia tensor
 
Param.kp = diag([15,15,10]);
Param.kv = diag([10,10,10]);
Param.ki = diag([0.01,0.01,0.01]);
Param.kR = diag([15,15,15]);
Param.kom= diag([1,1,1]);

% downwash
Param.arm_length = 0.35;
Param.L = 2 * Param.arm_length;
Param.Cax = 0.10;
Param.Crad = 0.30;
Param.z_0 = 0.330;

% rotor drag coeficient
dx = 0.18;
dy = 0.18;
dz = 0.18;
Param.D = diag ([dx, dy, dz]);



Param.d2_height = 2.0;


% initialize variables for all drones:
t = 0:Param.dTi:Param.Tend;
nt = length(t);
nx = 18;
nu = 4;
for iD = 1:Param.nD
    
    Nsim = Param.Nsim;
    
    if iD == 1
        % p0{iD} = [0;3;0.10*(0-15)^2+Param.d2_height+ Param.height_diff];
         p0{iD} = [0;3;7*exp(0)+Param.d2_height+ Param.height_diff];
        Param.p1 = p0{iD};
    end
    
    if iD == 2
        p0{iD} = [0;3;Param.d2_height];
        Param.p2 = p0{iD};
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
    

    
    if iD == 2 % linear for drone 2
        
        phase{iD} = (iD-1)*Param.dphase;
        
        p_ref{iD} = [s_factor*t ;3*ones(size(t));Param.d2_height*ones(size(t))];
        v_ref{iD} = [s_factor*ones(size(t));zeros(size(t));zeros(size(t))];
        % Drone parado
%         p_ref{iD} = [49*ones(size(t)) ;3*ones(size(t));Param.d2_height*ones(size(t))];
%         v_ref{iD} = [zeros(size(t));zeros(size(t));zeros(size(t))]
        a_ref{iD} = [zeros(size(t));zeros(size(t));zeros(size(t))];
        j_ref{iD} = [zeros(size(t));zeros(size(t));zeros(size(t))];
        psi_ref{iD} = atan2(v_ref{iD}(2,:),v_ref{iD}(1,:));
        dpsi_ref{iD} = 0*Param.omn*ones(size(psi_ref{iD}));
        vi{iD} = 0;
    end
    
    if iD == 1 % descendant trajectory
        
        
        a = 0.10;
        f = 7 * exp(-a*(t));

       
        p_ref{iD} = [s_factor*t; 3*ones(size(t)); f + Param.d2_height + Param.height_diff ];
        v_ref{iD} = [s_factor*ones(size(t));zeros(size(t)); -7*a*exp(-a*t)];
        a_ref{iD} = [zeros(size(t));zeros(size(t));  7*a^2*exp(-a*t) ];
        j_ref{iD} = [zeros(size(t));zeros(size(t));  -7*a^3*exp(a*t)];
        psi_ref{iD} = atan2(v_ref{iD}(2,:),v_ref{iD}(1,:));
        dpsi_ref{iD} = 0*Param.omn*ones(size(psi_ref{iD}));


%           a = 0.10;
%         p_ref{iD} = [t; 3*ones(size(t)); a*(t-15).^2 + Param.d2_height + Param.height_diff ];
%         v_ref{iD} = [ones(size(t));zeros(size(t)); 2*a*(t-15)];
%         a_ref{iD} = [zeros(size(t));zeros(size(t));  2*a*ones(size(t))];
%         j_ref{iD} = [zeros(size(t));zeros(size(t)); zeros(size(t))];
%         psi_ref{iD} = atan2(v_ref{iD}(2,:),v_ref{iD}(1,:));
%         dpsi_ref{iD} = 0*Param.omn*ones(size(psi_ref{iD}));

        
        
    end

p_ref_all{iD} = [p_ref{iD};v_ref{iD};a_ref{iD};j_ref{iD}];
psi_ref_all{iD} = [psi_ref{iD};dpsi_ref{iD}];
fprintf('Finished initializing trajectory for Drone %d \n', iD);
end