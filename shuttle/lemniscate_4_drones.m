
% general initialization
%clear all;
tracking_sim = 0;
Param.Tend = 30;
Param.dTi = 0.001;  % inner-loop and simulation sampling period

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
    
dcolors = { sstred, sstblue, yellow, sstlightgreen, sstlightestblue, sstlightgreen, sstlightergreen, sstlightgray };

% position plots of varying drone speeds ------
omn_values = [0.2,0.4,1,2];
counter = ['AA';'BB';'CC';'DD';'EE'];
word = '';
Param.omn = 0;

for i = 1:4
Param.Vw = [0; 0; 0];   
Param.omn  = omn_values(i);
drone_init_multp_simulations;
drone_simu_lemniscate;

is_wind = 0;
word = counter(i,:);
fprintf('\n\n Finished speed simulation %d \n', i);
show_data_scenarios;
end
disp(' \n WINNNNNND NOW -----------------------------------------');

%clear all;
% position plots of varying wind speeds ------
wind_values_x = [[0; 0; 0] ,[-2; 0; 0], [-5; 0; 0], [-10; 0; 0], [-15; 0; 0]];
counter = ['AA';'BB';'CC';'DD';'EE'];
word = '';

for i = 1:5
    
Param.Vw = wind_values_x(:,i);
Param.omn = 1;
drone_init_multp_simulations;
drone_simu_lemniscate;

is_wind = 1;
word = counter(i,:);
show_data_scenarios;
fprintf('\n Finished wind simulation %d', i);

end
