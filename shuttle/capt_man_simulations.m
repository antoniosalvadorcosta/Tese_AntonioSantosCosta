
% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% Summary: simulate capture maneuvre

clear all;

 set(0,'defaultTextInterpreter','latex');
    set(0,'defaultLegendInterpreter','latex');
    sstblue         = [0,128,255]/255;
    sstlightblue    = [48,208,216]/255;
    sstlighterblue  = [50,220,240]/255;
    sstlightestblue = [60,230,255]/255;
    sstgreen        = [43,191,92]/255;
    sstlightgreen   = [140,255,200]/255;
    sstlightergreen = [160,255,225]/255;
    sstgray         = [70,70,70]/255;
    sstlightgray    = [200,200,200]/255;
    sstred          = [255, 0, 0]/255;  % Vivid red
    sstbrown        = [165, 42, 42]/255; % Medium brown
    sstgray         = [70,70,70]/255;
    yellow          = [187, 139, 25]/255;
    %dcolors = { sstgreen, sstblue, sstred, sstlighterblue, sstlightestblue, sstlightgreen, sstlightergreen, sstlightgray };
    
    dcolors = { sstred, sstblue, yellow, sstbrown, sstlightestblue, sstlightgreen, sstlightergreen, sstlightgray };



%----------------------------- height difference above 2 rotor radius

% situation of the simulation of the 4 drones in drone_show_data (in this case should be 0);
situation = 0;

% drones safety distance
Param.height_diff = 0.5;

drone_init_6;

% flag for controller downwash compensation
dw_comp = 0;

drone_main_simul_cpte;

show_simulations_plots = 1;

drone_show_data;


% dcolors = { sstgreen, sstblue, sstred, sstlighterblue, sstlightestblue, sstlightgreen, sstlightergreen, sstlightgray };
% 
% % flag for controller downwash compensation
% dw_comp = 0;
% 
% drone_main_simul_cpte;
% 
% show_simulations_plots = 1;
% 
% drone_show_data;
