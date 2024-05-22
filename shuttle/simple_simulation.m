drone_init_7;



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
    %dcolors = { sstgreen, sstblue, sstlightblue, sstlighterblue, sstlightestblue, sstlightgreen, sstlightergreen, sstlightgray };
    
    dcolors = { sstred, sstblue, yellow, sstbrown, sstlightestblue, sstlightgreen, sstlightergreen, sstlightgray };


situation = 0;

drone_main_simul;

tracking_sim = 0;
show_simulations_plots = 1;

drone_show_data;