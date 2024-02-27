
% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% Summary: simulate capture maneuvre




clear all;

%----------------------------- height difference below 2 rotor radius

Param.height_diff = 0.30;

drone_init_6;

drone_main_simul_cpte;

show_simulations_plots = 1;

drone_show_data;

%----------------------------- height difference above 2 rotor radius

Param.height_diff = 0.40;

drone_init_6;

drone_main_simul_cpte;

show_simulations_plots = 0;

drone_show_data;

