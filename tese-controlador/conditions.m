%% instructions

% scenarios:

% 0.1 -> (leminscate) V1 - model with rotor drag; V2 - model and controller rotor drag
% 0.2 -> (lemniscate) V1 - basic controller;      V2 - model and controller rotor drag;

% 1   -> (lemniscate) V1 - basic controller;      V2 - basic controller + wind ?;
% 1.1 -> (lemniscate) V1 - full controller;      V2 - full controller + wind ;

% 4 - step basic controller
%% initiatlization on workspace
rd_on_model = 0;
rd_on_controller_model = 0;
wind_model = 0;
wind_model_controller = 0;

rd_on_model_2 = 0;
rd_on_controller_model_2 = 0;
wind_model_2 = 0;
wind_model_controller_2 = 0;


%% scenario for simulation

% 1 drone

scenario = 1.1; 

% 2 drones

%% drone 1
 

if scenario == 0.1
wind = 0;
trajectory = 1;
rd_on_model = 1;
rd_on_controller_model = 0;
end

if scenario == 0.2
wind = 0;
trajectory = 1;
rd_on_model = 0;
rd_on_controller_model = 0;
end 

% wind

if scenario == 1
wind_model = 0;
wind_model_controller = 0;
trajectory = 1;
rd_on_model = 0;
rd_on_controller_model = 0;
end

if scenario == 1.1
wind_model = 0;
wind_model_controller = 0;
trajectory = 1;
rd_on_model = 1;
rd_on_controller_model = 1;
end




%capture 
if scenario == 2
wind = 1;
trajectory = 4;
rd_on_model = 1;
rd_on_controller_model = 1;
end


% if scenario == 4   
% wind = 0;
% trajectory = 99;
% rd_on_model = 0;
% rd_on_controller_model = 0;
% end    

%% drone 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if scenario == 0.1
wind_model_2 = 0;
wind_model_controller_2 = 0;
trajectory_2 = 1;
rd_on_model_2 = 1;
rd_on_controller_model_2 = 1;
end


if scenario == 0.2
wind_model_2 = 0;
wind_model_controller_2 = 0;
trajectory_2 = 1;
rd_on_model_2 = 1;
rd_on_controller_model_2 = 1;
end

%wind 
if scenario == 1
wind_model_controller = 1;
trajectory_2 = 1;
rd_on_model_2 = 0;
rd_on_controller_model_2 = 0;
end

if scenario == 1.1
wind_model_controller = 1;
trajectory_2 = 1;
rd_on_model_2 = 1;
rd_on_controller_model_2 = 1;
end

if scenario == 2
wind_2 = 1;
trajectory_2 = 4;
rd_on_model_2 = 1;
rd_on_controller_model_2 = 1;
end


if scenario == 4   
wind_2= 0;
rd_on_model_2 = 1;
trajectory_2 = 99;
rd_on_controller_model_2 = 1;
end    
    
height = 0.15; % for scenario 2 - capture 
 
