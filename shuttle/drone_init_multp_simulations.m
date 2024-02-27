% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% inicializations

clear all;


% Model and simulation parameters
Param.Tend = 60;
Param.dTi = 0.001;  % inner-loop and simulation sampling period
Nsim = round(Param.Tend/Param.dTi)+1;
Param.g = 9.81;     % earth gravity
Param.nD = 4; % number of drones

% reference parameters
Param.p_ref_static = [0;0;0];
Param.psi_ref_static = pi/3;

Param.vz_d = 0.1;
Param.dh = 0.05;      % safety height difference between drones
Param.Rad = 5;        % radius of circle
Param.omn = 1;  % rotation frequency
Param.dphase = -pi/12;% ref circle angular difference between drones
Param.ref_mode = 2; % reference: 1 - square wave; 2 - circle
Param.Vw = [0;0;0];

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


omn = Param.omn;

% initialize variables for all drones:
t = 0:Param.dTi:Param.Tend;
nt = length(t);
nx = 18;
nu = 4;
for iD = 1:Param.nD
    

    
    
    % set initial conditions
    p0{iD} = [0;0;3];
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
    
    
    if Param.ref_mode == 1 % lemniscate
      
        b = 2;

        x_value = b*cos(t).*(1 + sin(t).^2);
        y_value = b*sin(t).*cos(t).*(1 + sin(t).^2);
        dot_x = -b*sin(t).*(3 + 2*sin(t).^2);
        dot_y = b*(cos(t).^2 - sin(t).^2).*(1 + sin(t).^2);
        dot_dot_x = -2*b*cos(t).*(3*sin(t).^4 + 4*sin(t).^2 + 1);
        dot_dot_y = -2*b*sin(t).*cos(t).*(3*sin(t).^4 - 4*sin(t).^2 + 1);
        dot_dot_dot_x = b*cos(t).*(27*sin(t).^6 + 32*sin(t).^4 + 16*sin(t).^2 - 4);
        dot_dot_dot_y = b*sin(t).*(27*sin(t).^6 + 32*sin(t).^4 - 16*sin(t).^2 - 4);
    end
    
    if Param.ref_mode == 2
        
        r = 1.5;
        
        x_value = 3 + r*cos(omn*t);
        y_value =  3 + r*sin(omn*t);
        dot_x = -r*omn*sin(omn*t);
        dot_y = r*omn*cos(omn*t);
        dot_dot_x = -r*omn^2*cos(omn*t);
        dot_dot_y =  -r*omn^2*sin(omn*t);
        dot_dot_dot_x = r*omn^3*sin(omn*t);
        dot_dot_dot_y = -r*omn^3*cos(omn*t);
    
    end
        
    
        
        if  iD == 2
            factor = 0;
        else
            factor = 1;
        end
        
        
        phase{iD} = (iD-1)*Param.dphase;
        p_ref{iD} = [x_value;y_value;3*ones(size(t))];
        v_ref{iD} = factor*[dot_x; dot_y ;zeros(size(t))];
        a_ref{iD} = factor*[dot_dot_x ; dot_dot_y ;zeros(size(t))];
        j_ref{iD} = factor*[ dot_dot_dot_x ;dot_dot_dot_y ;zeros(size(t))];
        psi_ref{iD} = atan2(v_ref{iD}(2,:),v_ref{iD}(1,:));
        dpsi_ref{iD} = 0*Param.omn*ones(size(psi_ref{iD}));
  
    
    
    
    
    
    
    
    p_ref_all{iD} = [p_ref{iD};v_ref{iD};a_ref{iD};j_ref{iD}];
    psi_ref_all{iD} = [psi_ref{iD};dpsi_ref{iD}];
    
  
    
end



