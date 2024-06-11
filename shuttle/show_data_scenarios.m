P = Param;
if situation == 1
    scenario_dcrpt = "4 scenarios (2,3,4,5) simulated at once";
    imgs_folder = 'figures/4_scenarios/allsc_';
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

legend('Reference', 'Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');

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
legend('Reference', 'Scenario A', 'Scenario B', 'Scenario C', 'Scenario D');
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

