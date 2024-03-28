
% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% Summary: simulate capture maneuvre

clear all;

%----------------------------- height difference below 2 rotor radius

% situation of the simulation of the 4 drones in drone_show_data (in this case should be 0);
situation = 0;

% Minimum height difference
Param.height_diff = 0.18;

% initialize both drones (iD = 1 and iD = 2)
drone_init_6;

% flag for controller downwash compensation
dw_comp = 0;

% simulation script
drone_main_simul_cpte;


% show or not all the plots option
show_simulations_plots = 1;

% show simulations script
drone_show_data;

%----------------------------- height difference above 2 rotor radius

% Param.height_diff = 3;
% 
% drone_init_6;
% 
% drone_main_simul_cpte;
% 
% show_simulations_plots = 0;
% 
% drone_show_data;

