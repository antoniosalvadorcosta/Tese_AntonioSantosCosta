% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)
%
% Summary: script that plots basic simulation results

 clf

P = Param;

Param.nD = 3;

global vd_store;
global vd_store_2;
global v_air_store;
global v_air_store_2;

if P.scenario == 1
    imgs_folder = 'simulacoes/scenario1/sc1_';
end

if P.scenario == 2
    scenario_dcrpt = "Naive Scenario";
    imgs_folder = 'simulacoes/scenario2/sc2_';
end

if P.scenario == 3
    scenario_dcrpt = "Feed Forward";
    imgs_folder = 'simulacoes/rotor_drag/sc3_';
end

if P.scenario == 4
    scenario_dcrpt = "Rotor Drag Compensation";
    imgs_folder = 'simulacoes/rotor_drag/sc4_';
end

if P.scenario > 4 && P.scenario ~= 6
    scenario_dcrpt = "Rotor Drag and Frame Drag Compensation";
    imgs_folder = 'simulacoes/rotor_drag/sc5_';
end

if P.scenario == 6
    scenario_dcrpt = "Capture maneuvre";
    imgs_folder = 'simulacoes/capt_man/sc6_';
end


if situation == 1
    scenario_dcrpt = "4 scenarios (2,3,4,5) simulated at once";
    imgs_folder = 'simulacoes/4_scenarios/allsc_';
end

saves_folder = 'saves/';
if ~exist('filename','var') || isempty(filename)
    %filename = [datestr(now,30) '_simul_'];
    filename = ['_simul_'];
end
do_print = 1; % set to 1 to plot each plot in a PDF file (for latex reports)
do_save_workspace = 0; % set to 1 to save workspace to *.mat file

% test if results are from script or simulink and prepare variables
% accordingly:
if exist('out','var')
    
    % test if there are two drones
    try
        if ~isfield(out.simout2,'time')
            Param.nD = 1;
        end
    catch
        Param.nD = 1;
    end
    
    for iD = 1:3
        if iD == 1
            t = out.simout1.time;
            data = permute(out.simout1.signals.values,[1,3,2]);
        else
            t = out.simout2.time;
            data = permute(out.simout2.signals.values,[1,3,2]);
        end
        
        [n1,n2] = size(data);
        if n1 > n2 % diferent Matlab versions use the transpose of the data values
            data = data';
        end
        ref = data((1:14),:);
        x{iD} = data(14+(1:18),:);
        u = data(14+18+(1:4),:);
        iep = data(14+18+4+(1:3),:);
        lbd{iD} = data(14+18+4+3+(1:3),:);
        
        p{iD} = x{iD}(1:3,:);
        
        p_ref{iD} = ref(1:3,:);
        v_ref{iD} = ref(4:6,:);
        psi_ref{iD} = ref(13,:);
        
        T{iD} = u(1,:);
        tau{iD} = u(2:4,:);
        
        
    end
    
end

% calculate and display rmse value

% wind simulations
if tracking_sim == 0 && P.scenario ~= 6 && P.scenario ~= 7
    wind_rmse_values_slice = [];
    e_p = p{1} - p_ref{1};
    %     for i = 1:3
    %         error = e_p(i,:);
    %         rmse_value =  sqrt(mean((error).^2, "all"));
    %         wind_rmse_values_slice = [wind_rmse_values_slice, rmse_value];
    %     end
    
    rmse_value =  sqrt(mean(vecnorm(e_p).^2));
    fprintf('\nRMSE "%s":\n',scenario_dcrpt);
    fprintf('%f\n',wind_rmse_values_slice);
    
    % other simulations
else
    
    if P.scenario == 6
        
        vd{1} = vd_store;
        vd{2} = vd_store_2;
        e_p = p{2} - p_ref{2};
        e_p2 = p_2{2} - p_ref{2};
        %z_ep = e_p(3,:);
        rmse_value =  sqrt(mean(vecnorm(e_p).^2));
        rmse_value2 =  sqrt(mean(vecnorm(e_p2).^2));
        
        z_ep = e_p(3,:);
        medium_error_z = mean(z_ep);
        z_ep2 = e_p2(3,:);
        medium_error_z2 = mean(z_ep2);
        
        fprintf('\nError in z (mean) "%s": ',scenario_dcrpt);
        fprintf('%f\n',medium_error_z);
        fprintf('%f\n',medium_error_z2);
        
        fprintf('\nRMSE in "%s": ',scenario_dcrpt);
        fprintf('%f\n',rmse_value);
        fprintf('%f\n',rmse_value2);
        
        fprintf('\n Max Thrust Basic: %f', max(T{2}));
        fprintf('\n Max Thrust Full: %f', max(T{3}));
        fprintf('\n Max vd Basic: %f', max(vd{1}));
        fprintf('\n Max vd Full: %f', max(vd{2}));
        
    else
        if P.scenario == 7
            e_p = p{1} - p_ref{1};
            z_ep = e_p(3,:);
            medium_error_z = mean(z_ep);
            fprintf('\nError in z (mean) "%s": ',scenario_dcrpt);
            fprintf('%f\n',medium_error_z);
        else
            e_p = p{1} - p_ref{1};
            rmse_value = sqrt(mean(vecnorm(e_p).^2));
            
            fprintf('\nRMSE "%s": ',scenario_dcrpt);
            fprintf('%f\n',rmse_value);
        end
        
    end
    
    
    
    
end

if show_simulations_plots ~= 0
    
    % show results plot
    %     set(0,'defaultTextInterpreter','latex');
    %     set(0,'defaultLegendInterpreter','latex');
    %     sstblue         = [0,128,255]/255;
    %     sstlightblue    = [48,208,216]/255;
    %     sstlighterblue  = [50,220,240]/255;
    %     sstlightestblue = [60,230,255]/255;
    %     sstgreen        = [43,191,92]/255;
    %     sstlightgreen   = [140,255,200]/255;
    %     sstlightergreen = [160,255,225]/255;
    %     sstgray         = [70,70,70]/255;
    %     sstlightgray    = [200,200,200]/255;
    %     sstred          = [255, 0, 0]/255;  % Vivid red
    %     sstbrown        = [165, 42, 42]/255; % Medium brown
    %     sstgray         = [70,70,70]/255;
    %     yellow          = [187, 139, 25]/255;
    %     %dcolors = { sstgreen, sstblue, sstlightblue, sstlighterblue, sstlightestblue, sstlightgreen, sstlightergreen, sstlightgray };
    %
    %dcolors = { sstred, sstgreen, sstblue, yellow, sstbrown, sstlightestblue, sstlightgreen, sstlightergreen, sstlightgray };
    
    nD = length(p);
    nt = length(t);
    dt = mean(t(2:end)-t(1:end-1));
    
    if situation == 0
        figure(100);
        title('Drones 3D trajectory');
        for iD = 1:Param.nD
            hini{iD} = plot3(p{iD}(1,1),p{iD}(2,1),p{iD}(3,1),'o','Color',dcolors{iD},'MarkerSize',2);
            if iD == 1, hold on; end
            %href{iD} = plot3(p_ref{iD}(1,:),p_ref{iD}(2,:),p_ref{iD}(3,:),'--','Color',sstgray);
            hp{iD} = plot3(p{iD}(1,:),p{iD}(2,:),p{iD}(3,:),'-','Color',dcolors{iD});
            %hd{iD} = drone_plot(p{iD}(1:3,1),lbd{iD}(:,1),[],dcolors{iD});
            last_p = p{iD}(1:3,1);
            for k = 2:10:nt
                dp = norm(p{iD}(1:3,k) - last_p);
                if dp > 3
    
                    %drone_plot(p{iD}(1:3,k),lbd{iD}(:,k),[],dcolors{iD});
                    last_p = p{iD}(1:3,k);
                end
            end
            %drone_plot(p{iD}(1:3,end),lbd{iD}(:,end),[],dcolors{iD});
        end
        hold off;
        %grid on;
        axis equal;
        % axis([-1.2 1.2 -1.2 1.2 0 3]);
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
        legend('Start Shutle Drone' ,'End Shutle Drone', 'Start Target Drone' ,'End Target Drone', 'Start Target Drone Full', ' End Target Drone Full');
        
        print2pdf([imgs_folder filename '_traj'],do_print);
        
    end
    
    % --------------------------------------- Control Variable --------------------
    figure(101);
    subplot(411);
    plot(t,T{1},'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,T{iD},'Color',dcolors{iD});
    end
    hold off;
    grid on;
    
    ylabel('$$T(t)$$ [N]');
%     if P.scenario == 6
%         legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
%     end
    title('Control variables');
    subplot(412);
    plot(t,tau{1}(1,:),'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,tau{iD}(1,:),'Color',dcolors{iD});
    end
    hold off;
    grid on;
    ylabel('$$\tau_1(t)$$ [N m]');
%     if P.scenario == 6
%         legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
%     end
    
    subplot(413);
    plot(t,tau{1}(2,:),'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,tau{iD}(2,:),'Color',dcolors{iD});
    end
    hold off;
    grid on;
    ylabel('$$\tau_2(t)$$ [N m]');
%     if P.scenario == 6
%         legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
%     end
    
    subplot(414);
    plot(t,tau{1}(3,:),'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,tau{iD}(3,:),'Color',dcolors{iD});
    end
    hold off;
    grid on;
    xlabel('$$t$$ [s]');
    ylabel('$$\tau_3(t)$$ [N m]');
    if P.scenario == 6
        legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
    end
    print2pdf([imgs_folder filename '_act'],do_print);
    
    
    %---------------------------------Position--------------------------------------------------------
    
    figure(102);
    subplot(311);
    
    hold on;
    plot(t,x{1}(1,:),'Color',dcolors{1});
    for iD = 2:Param.nD
        
        plot(t,x{iD}(1,:),'Color',dcolors{iD});
    end
    hold off;
    grid on;
    
%     legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
    
    ylabel('$$x(t)$$ [m]');
    
    title('Drone position');
    
    
    
    %------------------------- Position Y
    subplot(312);
    
    hold on;
    plot(t,x{1}(2,:),'Color',dcolors{1});
    for iD = 2:Param.nD
        if P.scenario ~= 6
            plot(t,p_ref{iD}(2,:),'Color',sstgray);
        end
        plot(t,x{iD}(2,:),'Color',dcolors{iD});
    end
    hold off;
    grid on;
%     if P.scenario ~= 6
%         legend('Y Reference', 'Y Position')
%     else
%         legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
%     end
    ylabel('$$y(t)$$ [m]');
    
    %------------------------- Position Z
    
    subplot(313);
    if P.scenario ~= 6
        plot(t,p_ref{1}(3,:),'Color',sstgray);
        
    end
    hold on;
    plot(t,x{1}(3,:),'Color',dcolors{1});
    for iD = 2:Param.nD
        if P.scenario ~= 6
            plot(t,p_ref{iD}(3,:),'Color',sstgray);
        end
        plot(t,x{iD}(3,:),'Color',dcolors{iD});
    end
    hold off;
    grid on;
    if P.scenario ~= 6
        legend('Z Reference', 'Z Position')
    else
        legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
    end
    xlabel('$$t$$ [s]');
    ylabel('$$z(t)$$ [m]');
    print2pdf([imgs_folder filename '_pos'],do_print);
    
    
    % disp("error:")
    % errorx= x{1}(1,:)-p_ref{1}(1,:);
    % disp(abs(errorx(length(errorx))));
    % errory= x{1}(2,:)-p_ref{1}(2,:);
    % disp(abs(errory(length(errory))));
    % errorz= x{1}(3,:)-p_ref{1}(3,:);
    % disp(abs(errorz(length(errorz))));
    
    %------------------------------ VELOCITY ----------------------------------
    figure(103);
    
    % Velocity in X direction
    subplot(311);
    %plot(t, v_ref{1}(1,:), 'Color', sstgray);
    hold on;
    plot(t, x{1}(4,:), 'Color', dcolors{1});
    for iD = 2:Param.nD
        %plot(t, v_ref{iD}(1,:), 'Color', sstgray);
        plot(t, x{iD}(4,:), 'Color', dcolors{iD});
    end
    hold off;
    grid on;
    ylabel('$$v_x(t)$$ [m]');
%     if P.scenario == 6
%         legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
%     end
    title('Drone velocity and reference');
    
    % Velocity in Y direction
    subplot(312);
    %plot(t, v_ref{1}(2,:), 'Color', sstgray);
    hold on;
    plot(t, x{1}(5,:), 'Color', dcolors{1});
    for iD = 2:Param.nD
        %plot(t, v_ref{iD}(2,:), 'Color', sstgray);
        plot(t, x{iD}(5,:), 'Color', dcolors{iD});
    end
    hold off;
    grid on;
    ylabel('$$v_y(t)$$ [m]');
%     if P.scenario == 6
%         legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
%     end
%     
    % Velocity in Z direction
    subplot(313);
    %plot(t, v_ref{1}(3,:), 'Color', sstgray);
    hold on;
    plot(t, x{1}(6,:), 'Color', dcolors{1});
    for iD = 2:Param.nD
        %plot(t, v_ref{iD}(3,:), 'Color', sstgray);
        plot(t, x{iD}(6,:), 'Color', dcolors{iD});
    end
    hold off;
    grid on;
    xlabel('$$t$$ [s]');
    ylabel('$$v_z(t)$$ [m]');
    if P.scenario == 6
        legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
    end
    
    print2pdf([imgs_folder filename '_vel'],do_print);
    
    %-------------------------------- Attitude -------------------------------
    
    figure(104);
    subplot(311);
    plot(t,lbd{1}(1,:)*180/pi,'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,lbd{iD}(1,:)*180/pi,'Color',dcolors{iD});
    end
    hold off;
    grid on;
    ylabel('$$\phi(t)$$ [deg]');
    title('Attitude (euler angles)');
%     if P.scenario == 6
%         legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
%     end
%     
    subplot(312);
    plot(t,lbd{1}(2,:)*180/pi,'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,lbd{iD}(2,:)*180/pi,'Color',dcolors{iD});
    end
    hold off;
    grid on;
%     if P.scenario == 6
%         legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
%     end
    ylabel('$$\theta(t)$$ [deg]');
    
    subplot(313);
    % plot(t,psi_ref{1}(1,:)*180/pi,'Color',sstgray);
    hold on;
    plot(t,lbd{1}(3,:)*180/pi,'Color',dcolors{1});
    for iD = 2:Param.nD
        % plot(t,psi_ref{iD}(1,:)*180/pi,'Color',sstgray);
        plot(t,lbd{iD}(3,:)*180/pi,'Color',dcolors{iD});
    end
    hold off;
    grid on;
    xlabel('$$t$$ [s]');
    ylabel('$$\psi(t)$$ [deg]');
    if P.scenario == 6
        legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
    end
    print2pdf([imgs_folder filename '_eul'],do_print);
    
    %--------------------------- Angular velocity ------------------------
    figure(105);
    subplot(311);
    plot(t,x{1}(16,:)*180/pi,'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,x{iD}(16,:)*180/pi,'Color',dcolors{iD});
    end
    hold off;
    grid on;
    ylabel('$$\omega_x(t)$$ [deg/s]');
%     if P.scenario == 6
%         legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
%     end
    title('Angular velocity');
    subplot(312);
    plot(t,x{1}(17,:)*180/pi,'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,x{iD}(17,:)*180/pi,'Color',dcolors{iD});
    end
    hold off;
    grid on;
    ylabel('$$\omega_y(t)$$ [deg/s]');
%     if P.scenario == 6
%         legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
%     end
    subplot(313);
    plot(t,x{1}(18,:)*180/pi,'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,x{iD}(18,:)*180/pi,'Color',dcolors{iD});
    end
    hold off;
    grid on;
    xlabel('$$t$$ [s]');
    ylabel('$$\omega_z(t)$$ [deg/s]');
    if P.scenario == 6
        legend('Shutle Drone', 'Target Drone', 'Target Drone Full');
    end
    print2pdf([imgs_folder filename '_om'],do_print);
    
    figure(109);
    %plot(t,psi_ref{1}(1,:)*180/pi,'Color',sstgray);
    hold on;
    plot(t,lbd{1}(3,:)*180/pi,'Color',dcolors{1});
    for iD = 2:Param.nD
        %plot(t,psi_ref{iD}(1,:)*180/pi,'Color',sstgray);
        plot(t,lbd{iD}(3,:)*180/pi,'Color',dcolors{iD});
    end
    hold off;
    grid on;
    xlabel('$$t$$ [s]');
    legend('Yaw Reference', 'Yaw Real')
    ylabel('$$\psi(t)$$ [deg]');
    % Set the plot limits to fill the plot area
    set(gca, 'XLim', [min(t), max(t)], 'YLim', [0, 130]);
    print2pdf([imgs_folder filename '_yaw'],do_print);
    %
    vd{1} = vd_store;
    vd{2} = vd_store_2;
    % downwash_3D_vect;
    if P.scenario == 6
        figure(4);
        
        hold on;
        for a = 1:2
            plot(t, vd{a}, 'Color', dcolors{a+1});
        end
        title('Downwash velocity');
        hold off;
        grid on;
        xlabel('$$t$$ [s]');
        legend('Target Drone', 'Target Drone Full')
        ylabel('$$v_{dw}(t)$$ [m/s]');
        print2pdf([imgs_folder filename '_dw'],do_print);
        
        
        
        figure(209);
        % Velocity in X direction
        subplot(311);
        hold on;
        plot(t, v_air_store(1,:), 'Color', dcolors{2});
        plot(t, v_air_store_2(1,:), 'Color', dcolors{3});
        hold off;
        grid on;
        ylabel('$$v_{air_x}(t)$$ [m/s]');
%         legend( 'Target Drone', 'Target Drone Full');
        title('Drones air ir relative velocity');
        
        % Velocity in Y direction
        subplot(312);
        hold on;
        plot(t, v_air_store(2,:), 'Color', dcolors{2});   
        plot(t, v_air_store_2(2,:), 'Color', dcolors{3});
        hold off;
        grid on;
        ylabel('$$v_{air_y}(t)$$ [m/s]');
%         legend( 'Target Drone', 'Target Drone Full');
       
        % Velocity in Z direction
        subplot(313);
        hold on;
        plot(t, v_air_store(3,:), 'Color', dcolors{2});   
        plot(t, v_air_store_2(3,:), 'Color', dcolors{3});
        hold off;
        grid on;
        xlabel('$$t$$ [s]');
        ylabel('$$v_{air_z}(t)$$ [m/s]');
        legend('Target Drone', 'Target Drone Full');
        print2pdf([imgs_folder filename '_air_vel'],do_print);
        
    end
    
    fprintf('\n Max Thrust Basic: %f', max(T{2}));
    fprintf('\n Max Thrust Full: %f', max(T{3}));
    fprintf('\n Max vd Basic: %f', max(vd{1}));
    fprintf('\n Max vd Full: %f', max(vd{2}));
    
    if do_save_workspace
        save([saves_folder filename '.mat']);
        disp(imgs_folder);
    end
    
    % if P.scenario < 6
    %     disp("Not displaying animation")
    %else
    % drone_animate(p,p_ref,lbd,t,dcolors);
    % end
end