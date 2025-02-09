P = Param;
 % Clear the figure
 clf



if situation == 1
    scenario_dcrpt = "4 scenarios (2,3,4,5) simulated at once";
    if is_wind == 0
        path = 'simulacoes/4_scenarios/speed/';
        imgs_folder = strcat(path, word);
        
    else
        path= 'simulacoes/4_scenarios/wind/';
        imgs_folder = strcat(path,word);
    end
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
    
    for iD = 1:Param.nD
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

% Plots ------------------

figure(1);
hini{iD} = plot3(p{iD}(1,1),p{iD}(2,1),p{iD}(3,1),'o','Color',dcolors{iD},'MarkerSize',2);
href{1} = plot3(p_ref{1}(1,:),p_ref{1}(2,:),p_ref{1}(3,:),'--','Color',sstgray);
for iD = 1:Param.nD
    if iD == 1, hold on; end
    hp{iD} = plot3(p{iD}(1,:),p{iD}(2,:),p{iD}(3,:),'-','Color',dcolors{iD});
end
hold off;
%grid on;
axis equal;

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
legend('Reference', 'Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');

% Reverse the y-axis direction
set(gca, 'YDir', 'reverse');

% Optionally, adjust other plot properties (e.g., axis limits, view angles)
axis equal;  % Equal aspect ratio
view(180, 90);  % View from upside-down perspective
print2pdf([imgs_folder filename '_4traj_view'],do_print);

%-------------------- Position 3D
figure(2);
%hini{iD} = plot3(p{iD}(1,1),p{iD}(2,1),p{iD}(3,1),'o','Color',dcolors{iD},'MarkerSize',2);
href{1} = plot3(p_ref{1}(1,:),p_ref{1}(2,:),p_ref{1}(3,:),'-','Color',sstgray);
for iD = 1:Param.nD
    if iD == 1, hold on; end
    hp{iD} = plot3(p{iD}(1,:),p{iD}(2,:),p{iD}(3,:),'-','Color',dcolors{iD});
end
hold off;
%grid on;
axis equal;
% axis([-1.2 1.2 -1.2 1.2 0 3]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
legend('Reference', 'Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');
print2pdf([imgs_folder filename '_4traj'],do_print);

% --------------------------------------- Control Variables --------------------
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

    
    subplot(413);
    plot(t,tau{1}(2,:),'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,tau{iD}(2,:),'Color',dcolors{iD});
    end
    hold off;
    grid on;
    ylabel('$$\tau_2(t)$$ [N m]');

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
    legend('Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');
    print2pdf([imgs_folder filename '_act'],do_print);
   
%-------------------- Position Coordinates
figure(3);
subplot(311);
plot(t,p_ref{1}(1,:),'Color',sstgray);
hold on;
plot(t,x{1}(1,:),'Color',dcolors{1});
for iD = 2:Param.nD
    
    %plot(t,p_ref{iD}(1,:),'Color',sstgray);
    plot(t,x{iD}(1,:),'Color',dcolors{iD});
end
hold off;
grid on;
ylabel('$$x(t)$$ [m]');
title('Drone position and reference');

%------------------------- Position Y
subplot(312);
plot(t,p_ref{1}(2,:),'Color',sstgray);
hold on;
plot(t,x{1}(2,:),'Color',dcolors{1});
for iD = 2:Param.nD
    %plot(t,p_ref{iD}(2,:),'Color',sstgray);
    plot(t,x{iD}(2,:),'Color',dcolors{iD});
end
hold off;
grid on;
% legend('Reference', 'Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');
ylabel('$$y(t)$$ [m]');

%------------------------- Position Z
subplot(313);
plot(t,p_ref{1}(3,:),'Color',sstgray);
hold on;
plot(t,x{1}(3,:),'Color',dcolors{1});
for iD = 2:Param.nD
    %plot(t,p_ref{iD}(3,:),'Color',sstgray);
    plot(t,x{iD}(3,:),'Color',dcolors{iD});
end
hold off;
grid on;
legend('Reference', 'Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');
xlabel('$$t$$ [s]');
ylabel('$$z(t)$$ [m]');
print2pdf([imgs_folder filename '_pos'],do_print);

for i = 1:4
    
    e_p = p{i} - p_ref{i};
    rmse_value = sqrt(mean(vecnorm(e_p).^2));
    
    fprintf(' RMSE %d: ',i);
    fprintf('%f',rmse_value);
end

%------------------------------ VELOCITY ----------------------------------
    figure(103);
    % Velocity in X direction
    subplot(311);
    plot(t, v_ref{1}(1,:), 'Color', sstgray);
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
    plot(t, v_ref{1}(2,:), 'Color', sstgray);
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
    plot(t, v_ref{1}(3,:), 'Color', sstgray);
    hold on;
    plot(t, x{1}(6,:), 'Color', dcolors{1});
    for iD = 2:Param.nD
        %plot(t, v_ref{iD}(3,:), 'Color', sstgray);
        plot(t, x{iD}(6,:), 'Color', dcolors{iD});
    end
    hold off;
    grid on;
    xlabel('$$t$$ [s]');
    ylabel('$$v_z(t)$$ [m]')
    legend('Reference', 'Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');
    print2pdf([imgs_folder filename '_vel'],do_print);

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

    legend( 'Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');
    
    print2pdf([imgs_folder filename '_om'],do_print);
    
    
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
 
    subplot(312);
    plot(t,lbd{1}(2,:)*180/pi,'Color',dcolors{1});
    hold on;
    for iD = 2:Param.nD
        plot(t,lbd{iD}(2,:)*180/pi,'Color',dcolors{iD});
    end
    hold off;
    grid on;
    ylabel('$$\theta(t)$$ [deg]');
    
    subplot(313);
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
    legend( 'Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');
    
    print2pdf([imgs_folder filename '_eul'],do_print);

% air velocity ------------------------------------------

figure(209);
% Velocity in X direction
subplot(311);
hold on;
plot(t, v_air{1}(1,:), 'Color', dcolors{1});
for iD = 2:Param.nD
        plot(t,v_air{iD}(1,:),'Color',dcolors{iD});
end
hold off;
grid on;
ylabel('$$v_{air_x}(t)$$ [m/s]');
title('Air ir relative velocity');

% Velocity in Y direction
subplot(312);
hold on;
plot(t, v_air{1}(2,:), 'Color', dcolors{1});
for iD = 2:Param.nD
        plot(t,v_air{iD}(2,:),'Color',dcolors{iD});
end
hold off;
grid on;
ylabel('$$v_{air_y}(t)$$ [m/s]');

% Velocity in Z direction
subplot(313);
hold on;
plot(t, v_air{1}(3,:), 'Color', dcolors{1});
for iD =2:Param.nD
        plot(t,v_air{iD}(3,:),'Color',dcolors{iD});
end
hold off;
grid on;
xlabel('$$t$$ [s]');
ylabel('$$v_{air_z}(t)$$ [m/s]');
legend( 'Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');
print2pdf([imgs_folder filename '_air_vel'],do_print);