 
 
kp = diag([75,100,100]);
kv = diag([12,16,28]);
kr = diag([4,4,3]);
kw = diag([0.1,0.1,0.1]);
 
 

% quase!!!
% kp = diag([150,1800,90]);
% kv = diag([13,220,10]);
% kr = diag([4,4,3]);
% kw = diag([0.1,0.1,0.1]);
 

%differential flatness
kh=0.009;

 
Tend = 16;
dT = 0.1;
N = round(Tend/dT)+1;
g = 9.8;
m = 0.5;

model = 0;
 

%Projected area "seen" by the wind
Proj_Area = [0.6 0 0;
             0 0.6 0;
             0 0 0.35];
         
Proj_Area2 = [1 1 0.5]';
         
         
%Area swept by the rotor
A = 0.025;

%air density
air_d = 1.3;

%drag coeficient
Cd = 1.15*10^-7; %visto na net: https://www.google.co.uk/search?q=drag+coeficient+for+quadrotors&tbm=isch&ved=2ahUKEwjNnLL226X4AhURLRoKHcDjBLUQ2-cCegQIABAA&oq=drag+coeficient+for+quadrotors&gs_lcp=CgNpbWcQAzoECCMQJzoECAAQQzoLCAAQgAQQsQMQgwE6CAgAEIAEELEDOgcIABCxAxBDOgUIABCABDoECAAQHjoGCAAQHhAIOgYIABAKEBhQojBYlXlgx3poAXAAeACAAbIBiAH9FpIBBDI0LjiYAQCgAQGqAQtnd3Mtd2l6LWltZ8ABAQ&sclient=img&ei=QrWkYs3zGJHaaMDHk6gL&bih=722&biw=1536#imgrc=Dz9DcbPm_-8BRM

%wind lateral angle xOy
wla = 30;

nx = 6;
nu = 4;
x = zeros(nx,N+1);
lbd = zeros(3,N+1);
u = zeros(nu,N);
% x(:,1) = [p0;v0];
% x_ref = [[0;0;0;0;0;0]*ones(1,1/dT+1),...
%             [0.5;0;1;0;0;0]*ones(1,(Tend-1)/dT)];
t = 0:dT:Tend;
 

p0 = [0;3;3];
v0 = [0;0;0];

 
 
%covert paremeters in to a structure
Parameter.g = g;
Parameter.m = m;

Parameter.kp = kp;
Parameter.kv = kv;
Parameter.kr = kr;
Parameter.kw = kw;
Parameter.kh = kh; 
Parameter.t = t;
Parameter.N = N; 
Parameter.A = A;
 
Parameter.Pa = Proj_Area;

%drag coeficient
Parameter.Cd = Cd;
Parameter.air_d = 1.3;

if scenario <= 1
Parameter.p_initial_1 = [0;0;3];
Parameter.p_initial_2 = [0;0;3];
end
if scenario == 2 
Parameter.p_initial_1 = [0;3;3];
Parameter.p_initial_2 = [0;3;92.51];
end
if scenario == 4
Parameter.p_initial_1 = [0;0;0];
Parameter.p_initial_2 = [0;0;0];  
end  

Parameter.static_p_ref = 0;%[[0;0;0]*ones(1,1/dT+1),...
        %  [0.5;0;1]*ones(1,(Tend-1)/dT)];

Parameter.v_initial = v0;
Parameter.R_initial = eye(3);
Parameter.omega_initial = [0;0;0]; %rotating a litle bit in each direction
Parameter.psi_dest = 0;
Parameter.dpsi_dest = 0;
Parameter.wla = wla;

Parameter.height = height;
%rotor geometry
Parameter.rotor_radius = 0.15;

%Inertia matrix
Parameter.I = diag([3.8e-3, 3.8e-3, 7.1e-3]);


%Fligth conditions


Parameter.rd_model = rd_on_model;
Parameter.rd_model_control = rd_on_controller_model;
Parameter.wind_model = wind_model;
Parameter.wind_model_controller = wind_model_controller;

 

Parameter.rd_model_2 = rd_on_model_2;
Parameter.rd_model_control_2 = rd_on_controller_model_2;
Parameter.wind_model = wind_model_2;
Parameter.wind_model_controller = wind_model_controller_2;

% 1 -> lemniscade
% 2 -> circle
% 3 -> spiral
Parameter.trajectory = trajectory;
Parameter.trajectory_2 = trajectory_2;


Parameter.scenario = scenario;
