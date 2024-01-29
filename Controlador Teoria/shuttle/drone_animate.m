function drone_animate(p,p_ref,lbd,t,dcolors)
% NOVA School of Science and Technology
% Department of Electrical and Computer Engineering
% IEEC course, fall 2021
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% Summary: script to animate drone based on simulation data

sstgray = [70,70,70]/255;
nD = length(p);
nt = length(t);
dt = mean(t(2:end)-t(1:end-1));
dk = round(max(1,0.05/dt));

figure(150);
for iD = 1:nD
    hini{iD} = plot3(p{iD}(1,1),p{iD}(2,1),p{iD}(3,1),'o','Color',dcolors{iD},'MarkerSize',2);
    if iD == 1, hold on; end
    href{iD} = plot3(p_ref{iD}(1,:),p_ref{iD}(2,:),p_ref{iD}(3,:),'--','Color',sstgray);
    hp{iD} = plot3(p{iD}(1,1:2),p{iD}(2,1:2),p{iD}(3,1:2),'-','Color',dcolors{iD});
    hd{iD} = drone_plot(p{iD}(1:3,1),lbd{iD}(:,1),[],dcolors{iD});
end
hold off;
grid on;
axis equal;
axis([-1.2 1.2 -1.2 1.2 0 3]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
legend('start','end','trajectory');
for k = 2:dk:nt
    for iD = 1:nD
        set(hp{iD},'xdata',p{iD}(1,1:k),'ydata',p{iD}(2,1:k),'zdata',p{iD}(3,1:k));
        drone_plot(p{iD}(1:3,k),lbd{iD}(:,k),hd{iD});
    end
    axis equal;
    drawnow;
    %pause(dt/10);
end
