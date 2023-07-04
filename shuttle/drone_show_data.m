% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)
% 
% Summary: script that plots basic simulation results

P = Param;

if P.scenario == 1
imgs_folder = 'figures/mellinger/sc1_';
end

if P.scenario == 2
imgs_folder = 'figures/mellinger/sc2_';
end

if P.scenario == 3
imgs_folder = 'figures/rotor_drag/sc3_';
end

if P.scenario == 4
imgs_folder = 'figures/rotor_drag/sc4_';
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

% show results plot
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

dcolors = { sstgreen, sstblue, sstlightblue, sstlighterblue, sstlightestblue, sstlightgreen, sstlightergreen, sstlightgray };

sstgray = [70,70,70]/255;
nD = length(p);
nt = length(t);
dt = mean(t(2:end)-t(1:end-1));

figure(100);
for iD = 1:Param.nD
    hini{iD} = plot3(p{iD}(1,1),p{iD}(2,1),p{iD}(3,1),'o','Color',dcolors{iD},'MarkerSize',2);
    if iD == 1, hold on; end
    href{iD} = plot3(p_ref{iD}(1,:),p_ref{iD}(2,:),p_ref{iD}(3,:),'--','Color',sstgray);
    hp{iD} = plot3(p{iD}(1,:),p{iD}(2,:),p{iD}(3,:),'-','Color',dcolors{iD});
    hd{iD} = drone_plot(p{iD}(1:3,1),lbd{iD}(:,1),[],dcolors{iD});
    last_p = p{iD}(1:3,1);
    for k = 2:10:nt
        dp = norm(p{iD}(1:3,k) - last_p);
        if dp > 3
            drone_plot(p{iD}(1:3,k),lbd{iD}(:,k),[],dcolors{iD});
            last_p = p{iD}(1:3,k);
        end
    end
    drone_plot(p{iD}(1:3,end),lbd{iD}(:,end),[],dcolors{iD});
end
hold off;
grid on;
axis equal;
% axis([-1.2 1.2 -1.2 1.2 0 3]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
legend('start','end','trajectory','Location','southeast');
print2pdf([imgs_folder filename '_traj'],do_print);

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
print2pdf([imgs_folder filename '_act'],do_print);
%---------------------------------Position--------------------------------------------------------
figure(102);
subplot(311);
plot(t,p_ref{1}(1,:),'Color',sstgray);
hold on;
plot(t,x{1}(1,:),'Color',dcolors{1});
for iD = 2:Param.nD
    plot(t,p_ref{iD}(1,:),'Color',sstgray);
    plot(t,x{iD}(1,:),'Color',dcolors{iD});
end
hold off;
grid on;
legend('X Reference', 'X Position')
ylabel('$$x(t)$$ [m]');
title('Drone position and reference');
subplot(312);
plot(t,p_ref{1}(2,:),'Color',sstgray);
hold on;
plot(t,x{1}(2,:),'Color',dcolors{1});

for iD = 2:Param.nD
    plot(t,p_ref{iD}(2,:),'Color',sstgray);
    plot(t,x{iD}(2,:),'Color',dcolors{iD});
end
hold off;
grid on;
legend('Y Reference', 'Y Position')
ylabel('$$y(t)$$ [m]');
subplot(313);
plot(t,p_ref{1}(3,:),'Color',sstgray);
hold on;
plot(t,x{1}(3,:),'Color',dcolors{1});

for iD = 2:Param.nD
    plot(t,p_ref{iD}(3,:),'Color',sstgray);
    plot(t,x{iD}(3,:),'Color',dcolors{iD});
end
hold off;
grid on;
legend('Z Reference', 'Z Position')
xlabel('$$t$$ [s]');
ylabel('$$z(t)$$ [m]');
print2pdf([imgs_folder filename '_pos'],do_print);

%------------------------------ VELOCITY ----------------------------------
figure(103);
subplot(311);
plot(t,v_ref{1}(1,:),'Color',sstgray);
hold on;
plot(t,x{1}(4,:),'Color',dcolors{1});

for iD = 2:Param.nD
    plot(t,v_ref{iD}(1,:),'Color',sstgray);
    plot(t,x{iD}(4,:),'Color',dcolors{iD});
end
hold off;
grid on;
ylabel('$$v_x(t)$$ [m]');
title('Drone velocity and reference');
subplot(312);
plot(t,v_ref{1}(2,:),'Color',sstgray);
hold on;
plot(t,x{1}(5,:),'Color',dcolors{1});
for iD = 2:Param.nD
    plot(t,v_ref{iD}(2,:),'Color',sstgray);
    plot(t,x{iD}(5,:),'Color',dcolors{iD});
end
hold off;
grid on;
ylabel('$$v_y(t)$$ [m]');
subplot(313);
plot(t,v_ref{1}(3,:),'Color',sstgray);
hold on;
plot(t,x{1}(6,:),'Color',dcolors{1});
for iD = 2:Param.nD
    plot(t,v_ref{iD}(3,:),'Color',sstgray);
    plot(t,x{iD}(6,:),'Color',dcolors{iD});
end
hold off;
grid on;
xlabel('$$t$$ [s]');
ylabel('$$v_z(t)$$ [m]');
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
plot(t,psi_ref{1}(1,:)*180/pi,'Color',sstgray);
hold on;
plot(t,lbd{1}(3,:)*180/pi,'Color',dcolors{1});
for iD = 2:Param.nD
    plot(t,psi_ref{iD}(1,:)*180/pi,'Color',sstgray);
    plot(t,lbd{iD}(3,:)*180/pi,'Color',dcolors{iD});
end
hold off;
grid on;
xlabel('$$t$$ [s]');
ylabel('$$\psi(t)$$ [deg]');
print2pdf([imgs_folder filename '_eul'],do_print);

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
print2pdf([imgs_folder filename '_om'],do_print);

if do_save_workspace
    save([saves_folder filename '.mat']);
end

drone_animate(p,p_ref,lbd,t,dcolors);
