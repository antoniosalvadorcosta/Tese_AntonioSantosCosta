function drone_animate(p, p_ref, lbd, t, dcolors)
sstgray = [70,70,70]/255;
nD = length(p);
nt = length(t);
dt = mean(t(2:end)-t(1:end-1));
dk = round(max(1,0.05/dt));

f = figure(150);
set(f, 'Color', 'w');  % Set the background color to white
 

for iD = 1:nD
    hini{iD} = plot3(p{iD}(1,1),p{iD}(2,1),p{iD}(3,1),'o','Color',dcolors{iD},'MarkerSize',2);
    if iD == 1, hold on; end
    if iD == 3, continue; end
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
%legend('start','end','trajectory');

% Initialize the video writer
filename = 'ani.avi';
v = VideoWriter(filename, 'Uncompressed AVI');
%v.FrameRate = (1/dt);  % Set the frame rate to double the original
open(v);

% Preallocate the structure array for frames
F(nt) = struct('cdata', [], 'colormap', []);

for k = 2:dk:nt
    for iD = 1:nD
        if iD == 3, continue; end
        set(hp{iD},'xdata',p{iD}(1,1:k),'ydata',p{iD}(2,1:k),'zdata',p{iD}(3,1:k));
        drone_plot(p{iD}(1:3,k),lbd{iD}(:,k),hd{iD});
    end
    axis equal;
    drawnow;
    
    % Capture the plot as an image
    F(k) = getframe(f);
    writeVideo(v, F(k));
end

close(v);
end
