
% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)
%
% Summary: script that plots basic simulation results

P = Param;

Param.nD = 3;

global vd_store;
global vd_store_2;
global v_air_store;
global v_air_store_2;

if P.scenario == 1
    imgs_folder = 'figures/mellinger/sc1_';
end

if P.scenario == 2
    scenario_dcrpt = "Naive Scenario";
    imgs_folder = 'figures/mellinger/sc2_';
end

if P.scenario == 3
    scenario_dcrpt = "Feed Forward";
    imgs_folder = 'figures/rotor_drag/sc3_';
end

if P.scenario == 4
    scenario_dcrpt = "Rotor Drag Compensation";
    imgs_folder = 'figures/rotor_drag/sc4_';
end

if P.scenario > 4 && P.scenario ~= 6
    scenario_dcrpt = "Rotor Drag and Frame Drag Compensation";
    imgs_folder = 'figures/rotor_drag/sc5_';
end

if P.scenario == 6
    scenario_dcrpt = "Capture maneuvre";
    imgs_folder = 'figures/capture_maneuvre/sc6_';
end


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



%--------------------------------- Control Variables --------------------
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
legend('Shuttle Drone - Control Thrust', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');
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
legend('Shuttle Drone - Torque \tau_1', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');

subplot(413);
plot(t,tau{1}(2,:),'Color',dcolors{1});
hold on;
for iD = 2:Param.nD
    plot(t,tau{iD}(2,:),'Color',dcolors{iD});
end
hold off;
grid on;
ylabel('$$\tau_2(t)$$ [N m]');
legend('Shuttle Drone - Torque \tau_2', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');

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
legend('Shuttle Drone - Torque \tau_3', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');
print2pdf([imgs_folder filename '_act'],do_print);

%--------------------------------- Position ------------------------------
figure(102);
subplot(311);
plot(t,x{1}(1,:),'Color',dcolors{1});
hold on;
for iD = 2:Param.nD
    plot(t,x{iD}(1,:),'Color',dcolors{iD});
end
hold off;
grid on;
legend('Shuttle Drone - x Position', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');
ylabel('$$x(t)$$ [m]');
title('Drone position');

subplot(312);
hold on;
plot(t,x{1}(2,:),'Color',dcolors{1});
for iD = 2:Param.nD
    plot(t,x{iD}(2,:),'Color',dcolors{iD});
end
hold off;
grid on;
legend('Shuttle Drone - y Position', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');
ylabel('$$y(t)$$ [m]');

subplot(313);
hold on;
plot(t,x{1}(3,:),'Color',dcolors{1});
for iD = 2:Param.nD
    plot(t,x{iD}(3,:),'Color',dcolors{iD});
end
hold off;
grid on;
legend('Shuttle Drone - z Position', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');
xlabel('$$t$$ [s]');
ylabel('$$z(t)$$ [m]');
print2pdf([imgs_folder filename '_pos'],do_print);

%--------------------------------- Velocity ------------------------------
figure(103);
subplot(311);
hold on;
plot(t,x{1}(4,:),'Color',dcolors{1});
for iD = 2:Param.nD
    plot(t,x{iD}(4,:),'Color',dcolors{iD});
end
hold off;
grid on;
ylabel('$$v_x(t)$$ [m/s]');
legend('Shuttle Drone - v_x Velocity', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');
title('Drone velocity');

subplot(312);
hold on;
plot(t,x{1}(5,:),'Color',dcolors{1});
for iD = 2:Param.nD
    plot(t,x{iD}(5,:),'Color',dcolors{iD});
end
hold off;
grid on;
ylabel('$$v_y(t)$$ [m/s]');
legend('Shuttle Drone - v_y Velocity', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');

subplot(313);
hold on;
plot(t,x{1}(6,:),'Color',dcolors{1});
for iD = 2:Param.nD
    plot(t,x{iD}(6,:),'Color',dcolors{iD});
end
hold off;
grid on;
xlabel('$$t$$ [s]');
ylabel('$$v_z(t)$$ [m/s]');
legend('Shuttle Drone - v_z Velocity', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');
print2pdf([imgs_folder filename '_vel'],do_print);

%--------------------------------- Attitude ------------------------------
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
title('Attitude (Euler angles)');
legend('Shuttle Drone - \phi', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');

subplot(312);
plot(t,lbd{1}(2,:)*180/pi,'Color',dcolors{1});
hold on;
for iD = 2:Param.nD
    plot(t,lbd{iD}(2,:)*180/pi,'Color',dcolors{iD});
end
hold off;
grid on;
ylabel('$$\theta(t)$$ [deg]');
legend('Shuttle Drone - \theta', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');

subplot(313);
hold on;
plot(t,lbd{1}(3,:)*180/pi,'Color',dcolors{1});
for iD = 2:Param.nD
    plot(t,lbd{iD}(3,:)*180/pi,'Color',dcolors{iD});
end
hold off;
grid on;
xlabel('$$t$$ [s]');
ylabel('$$\psi(t)$$ [deg]');
legend('Shuttle Drone - \psi', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');
print2pdf([imgs_folder filename '_eul'],do_print);

%-------------------------------- Angular Velocity -----------------------
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
legend('Shuttle Drone - \omega_x', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');

subplot(312);
plot(t,x{1}(17,:)*180/pi,'Color',dcolors{1});
hold on;
for iD = 2:Param.nD
    plot(t,x{iD}(17,:)*180/pi,'Color',dcolors{iD});
end
hold off;
grid on;
ylabel('$$\omega_y(t)$$ [deg/s]');
legend('Shuttle Drone - \omega_y', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');

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
legend('Shuttle Drone - \omega_z', 'Target Drone (No Compensation)', 'Target Drone (With Compensation)');
print2pdf([imgs_folder filename '_angvel'],do_print);
