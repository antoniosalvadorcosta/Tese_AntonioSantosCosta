
% this code written for making the animation of Quadcoptor model, all units
% are in meters, in this code of example we are using 'HGtransform'
% function for animate the trajectory of quadcopter

 %% 1. Simulation
conditions;  
initialization;
out = sim('quadcopter_dynamics2.slx');



 %% 2. define the motion coordinates 
 
 
    %simulation values drone 1
    yaw   =  out.simout.Data(:,9); %verificar ordem
    roll  = out.simout.Data(:,7);
    pitch = out.simout.Data(:,8);
    x1 = out.simout.Data(:,4);
    y1 = out.simout.Data(:,5);
    z1 = out.simout.Data(:,6);
 
    error_x = out.simout.Data(:,13);
    error_y = out.simout.Data(:,14);
    error_z = out.simout.Data(:,15);
    u1 = out.simout.Data(:,16);
    
    %simulation values drone 2
    yaw2  =  out.simout1.Data(:,9); %verificar ordem
    roll2  = out.simout1.Data(:,7);
    pitch2 = out.simout1.Data(:,8);
    x2 = out.simout1.Data(:,4);
    y2 = out.simout1.Data(:,5);
    z2 = out.simout1.Data(:,6);
    
    error_x_2 = out.simout1.Data(:,13);
    error_y_2 = out.simout1.Data(:,14);
    error_z_2 = out.simout1.Data(:,15);
    
    u1_2 = out.simout1.Data(:,16);
    felted_downwash_x = out.simout.Data(:,17);
    felted_downwash_y = out.simout.Data(:,18);
%     felted_downwash_z = out.simout.Data(:,19);
    
    %reference values
    tt_ref    = out.simout.Time;
    z_ref     =  out.simout.Data(:,12);
    y_ref     =  out.simout.Data(:,11);
    x_ref     =  out.simout.Data(:,10);
    

    yaw_ref   = 0; %1.2*tt_ref;
    roll_ref  = 0; %5*sin(5*tt_ref);
    pitch_ref = 0; %5*cos(5*tt_ref);
    
    
    
    %reference values drone 2
     
    z_ref2     =  out.simout1.Data(:,12);
    y_ref2     =  out.simout1.Data(:,11);
    x_ref2     =  out.simout1.Data(:,10);
    yaw_ref2   = 0; %1.2*tt_ref;
    roll_ref2  = 0; %5*sin(5*tt_ref);
    pitch_ref2 = 0; %5*cos(5*tt_ref);
    
    % second drone
%     if Parameter.trajectory_2 == 4
%         for i = 1:length(x_ref)
%             z2(i) =  exp((0.5*(tt_ref(i)-10)^2)/5^2)+2.5;
%         end
%     end
  

 %% 6. animate by using the function makehgtform
 % Function for ANimation of QuadCopter
  %anima(x1,y1,z1,roll,pitch,yaw,x2,y2,z2);

  
  %% 7.Graphs
 

% position 


if scenario == 0.1
    
    %3D plot
    figure(1);
    plot3(x1 ,y1 , z1, '-b');
    hold on;
    plot3(x_ref,y_ref, z_ref', '-g');
    hold on;
    plot3(x2,y2,z2', '-r');
    title('Lemniscate Trajectory');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    legend('Vehicle 1','Reference trajectory','Vehicle 2');
    print2pdf('position_lemniscate_rotorDrag_model_vs_model_control',1);
    
    %Position--------------------
    figure(6);
    subplot(1,3,1);
    title('Position in X');
    plot(tt_ref,x1,'-b');
    hold on;
    plot(tt_ref,x_ref,'-g');
    hold on;
    plot(tt_ref,x2,'-r');
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','x_{ref}','Vehicle 2');
  
    subplot(1,3,2); 
    plot(tt_ref,y1,'-b');
    title('Position in Y');
    hold on;
    plot(tt_ref,y_ref,'-g');
    hold on;
    grid on;
    plot(tt_ref,y2,'-r');
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','y_{ref}','Vehicle 2');
     
    subplot(1,3,3);
    plot(tt_ref,z1,'-b');
    title('Position in Z');
    hold on;
    plot(tt_ref,z_ref,'-g');
    hold on;
    plot(tt_ref,z2,'-r');
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','z_{ref}','Vehicle 2');
    print2pdf('position_lemniscate_rotorDrag',1);
    
    %Error ------------------
    figure(7);
    subplot(1,3,1);
    title('Position error in X');
    plot(tt_ref,error_x,'-b');
    hold on;
    plot(tt_ref,error_x_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
  
    subplot(1,3,2); 
    plot(tt_ref,error_y,'-b');
    title('Position error in Y');
    hold on;
    plot(tt_ref,error_y_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
     
    subplot(1,3,3);
    plot(tt_ref,error_z,'-b');
    title('Position error in Z');
    hold on;
    plot(tt_ref,error_z_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
    print2pdf('error_lemniscate_rotorDrag_model_vs_model_controller',1);
    
    %------------------------------------------------------
    
    
    
end

if scenario == 0.2
    
    %3D plot
    figure(1);
    plot3(x1 ,y1 , z1, '-b');
    hold on;
    plot3(x_ref,y_ref, z_ref', '-g');
    hold on;
    plot3(x2,y2,z2', '-r');
    title('Lemniscate Trajectory');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    legend('Vehicle 1','Reference trajectory','Vehicle 2');
    print2pdf('position_lemniscate_basic_vs_model_control',1);
    
    %Position--------------------
    figure(6);
    subplot(1,3,1);
    title('Position in X');
    plot(tt_ref,x1,'-b');
    hold on;
    plot(tt_ref,x_ref,'-g');
    hold on;
    plot(tt_ref,x2,'-r');
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','x_{ref}','Vehicle 2');
  
    subplot(1,3,2); 
    plot(tt_ref,y1,'-b');
    title('Position in Y');
    hold on;
    plot(tt_ref,y_ref,'-g');
    hold on;
    grid on;
    plot(tt_ref,y2,'-r');
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','y_{ref}','Vehicle 2');
     
    subplot(1,3,3);
    plot(tt_ref,z1,'-b');
    title('Position in Z');
    hold on;
    plot(tt_ref,z_ref,'-g');
    hold on;
    plot(tt_ref,z2,'-r');
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','z_{ref}','Vehicle 2');
    print2pdf('position_lemniscate_basic_vs_rotorDrag_model_control',1);
    
    %Error ------------------
    figure(7);
    subplot(1,3,1);
    title('Position error in X');
    plot(tt_ref,error_x,'-b');
    hold on;
    plot(tt_ref,error_x_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
  
    subplot(1,3,2); 
    plot(tt_ref,error_y,'-b');
    title('Position error in Y');
    hold on;
    plot(tt_ref,error_y_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
     
    subplot(1,3,3);
    plot(tt_ref,error_z,'-b');
    title('Position error in Z');
    hold on;
    plot(tt_ref,error_z_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
    print2pdf('error_lemniscate_basic_vs_rotorDrag_model_control',1);
    
    %------------------------------------------------------
    
    
    
end

if scenario == 1
    
    %3D plot
    figure(1);
    plot3(x1 ,y1 , z1, '-b');
    hold on;
    plot3(x_ref,y_ref, z_ref', '-g');
    hold on;
    plot3(x2,y2,z2', '-r');
    title('Lemniscate Trajectory');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    legend('Vehicle 1','Reference trajectory','Vehicle 2');
    print2pdf('position_lemniscate_basic_vs_wind',1);
    
    %Position--------------------
    figure(6);
    subplot(1,3,1);
    title('Position in X');
    plot(tt_ref,x1,'-b');
    hold on;
    plot(tt_ref,x_ref,'-g');
    hold on;
    plot(tt_ref,x2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','x_{ref}','Vehicle 2');
  
    subplot(1,3,2); 
    plot(tt_ref,y1,'-b');
    title('Position in Y');
    hold on;
    plot(tt_ref,y_ref,'-g');
    hold on;
    grid on;
    plot(tt_ref,y2,'-r');
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','y_{ref}','Vehicle 2');
     
    subplot(1,3,3);
    plot(tt_ref,z1,'-b');
    title('Position in Z');
    hold on;
    plot(tt_ref,z_ref,'-g');
    hold on;
    plot(tt_ref,z2,'-r');
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','z_{ref}','Vehicle 2');
    print2pdf('position_lemniscate_basic_vs_wind',1);
    
    %Error ------------------
    figure(7);
    subplot(1,3,1);
    title('Position error in X');
    plot(tt_ref,error_x,'-b');
    hold on;
    plot(tt_ref,error_x_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
  
    subplot(1,3,2); 
    plot(tt_ref,error_y,'-b');
    title('Position error in Y');
    hold on;
    plot(tt_ref,error_y_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
     
    subplot(1,3,3);
    plot(tt_ref,error_z,'-b');
    title('Position error in Z');
    hold on;
    plot(tt_ref,error_z_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
    print2pdf('error_lemniscate_basic_vs_wind',1);
    
    %------------------------------------------------------
    
    
    
end

if scenario == 1.1
    
    %3D plot
    figure(1);
    plot3(x1 ,y1 , z1, '-b');
    hold on;
    plot3(x_ref,y_ref, z_ref', '-g');
    hold on;
    plot3(x2,y2,z2', '-r');
    title('Lemniscate Trajectory');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    legend('Vehicle 1','Reference trajectory','Vehicle 2');
    print2pdf('position_lemniscate_full_vs_wind',1);
    
    %Position--------------------
    figure(6);
    subplot(1,3,1);
    title('Position in X');
    plot(tt_ref,x1,'-b');
    hold on;
    plot(tt_ref,x_ref,'-g');
    hold on;
    plot(tt_ref,x2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','x_{ref}','Vehicle 2');
  
    subplot(1,3,2); 
    plot(tt_ref,y1,'-b');
    title('Position in Y');
    hold on;
    plot(tt_ref,y_ref,'-g');
    hold on;
    grid on;
    plot(tt_ref,y2,'-r');
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','y_{ref}','Vehicle 2');
     
    subplot(1,3,3);
    plot(tt_ref,z1,'-b');
    title('Position in Z');
    hold on;
    plot(tt_ref,z_ref,'-g');
    hold on;
    plot(tt_ref,z2,'-r');
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('Vehicle 1','z_{ref}','Vehicle 2');
    print2pdf('position_lemniscate_full_vs_wind',1);
    
    %Error ------------------
    figure(7);
    subplot(1,3,1);
    title('Position error in X');
    plot(tt_ref,error_x,'-b');
    hold on;
    plot(tt_ref,error_x_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
  
    subplot(1,3,2); 
    plot(tt_ref,error_y,'-b');
    title('Position error in Y');
    hold on;
    plot(tt_ref,error_y_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
     
    subplot(1,3,3);
    plot(tt_ref,error_z,'-b');
    title('Position error in Z');
    hold on;
    plot(tt_ref,error_z_2,'-r');
    hold on;
    grid on;
    xlabel('Time(s)');
    ylabel('Error(m)');
    legend('Vehicle 1','Vehicle 2');
    print2pdf('error_lemniscate_full_vs_wind',1);
    
    %------------------------------------------------------
    
    
    
end



if scenario == 2
%     figure(1);
%     plot3(x1 ,y1 , z1, '-b');
%     hold on;
%     plot3(x2,y2, z2, '-r');
%     grid on;
%     title('Position');
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
%     axis equal;
%     legend('Vehicle 1', 'Vehicle 2');
% %      print2pdf('trajetoria_lemniscate_3D',1);
%     
%     
%     figure(6);
%     plot(tt_ref,x1,'-b');
%     hold on;
%     plot(tt_ref,x2,'-r');
%     grid on;
%     xlabel('Time(s)');
%     ylabel('Position(m)');
%     legend('Vehicle 1','Vehicle 2');
% %      print2pdf('trajetoria_lemniscate_x',1);
%     
%     figure(3);
%     plot(tt_ref,y1,'-b');
%     hold on;
%     plot(tt_ref,y2,'-r');
%     grid on;
%     xlabel('Time(s)');
%     ylabel('Position(m)');
%     legend('Vehicle 1','Vehicle 2');
% %      print2pdf('trajetoria_lemniscate_y',1);
%     
%     figure(4);
%     plot(tt_ref,z1,'-b');
%     hold on;
%     plot(tt_ref,z2,'-r');
%     grid on;
%     xlabel('Time(s)');
%     ylabel('Position(m)');
%     legend('Vehicle 1','Vehicle 2');
% %     print2pdf('trajetoria_lemniscate_z',1);
% %     
%     
% %     figure(5);
% %     plot (tt_ref,yaw,'-d');
% %     hold on;
% %     plot (tt_ref,yaw2,'-r');
% %     xlabel('Time(s)');
% %     ylabel('Yaw (Degrees)');
% %     legend('Vehicle 1', 'Vehicle 2');
% % %     print2pdf('yaw_lemniscate',1);
% %     
% %     figure(7);
% %     plot (tt_ref,roll,'-d');
% %     hold on;
% %     plot (tt_ref,roll2,'-r');
% %     xlabel('Time(s)');
% %     ylabel('Rol (Degrees)');
% %     legend('Vehicle 1', 'Vehicle 2');
% % %     print2pdf('roll_lemniscate',1);
% %     
% %     figure(8);
% %     plot (tt_ref,pitch,'-d');
% %     hold on;
% %     plot (tt_ref,pitch2,'-r');
% %     xlabel('Time(s)');
% %     ylabel('Pitch (Degrees)');
% %     legend('Vehicle 1', 'Vehicle 2');
% % %     print2pdf('Pitch_lemniscate',1);
%     
%     
%     figure(9);
%     plot (tt_ref,u1,'-d');
%     hold on;
%     plot (tt_ref,u1_2,'-r');
%     xlabel('Time(s)');
%     ylabel('Control');
%     legend('Vehicle 1', 'Vehicle 2');
% %     print2pdf('control_lemniscate',1);
% 
% 
% %   figure(10);
% %  streamtube([felted_downwash_x,felted_downwash_z]);
%  % quiver(felted_downwash_x,felted_downwash_y,felted_downwash_z);
%  hold on;

%Function downwash study
R = 0.15;
k = 5;
h = 1;
zr = 3.2;
 
z = 2.8:0.01:3.2;
vi =  sqrt(u1_2(1)/(2*1.3*0.025));
wc = (vi*(1+tanh(-k*(zr-z)/h)))/46.7485;
inds = (0.5 < 0.15./(sqrt(1+tanh(-k*((zr-z)/h)))));

k2 = 5;
h2 = 1.5;

wc2 =  vi*(1+tanh(-k2*(zr-z)/h2));
inds2 = (0.5 < 0.15./(sqrt(1+tanh(-k2*((zr-z)/h2)))));

wc(inds) = 0;
wc2(inds2) = 0;

figure(17);

plot(z, wc, 'b');
hold on;
xlabel('z(m)');
ylabel('Downwash(m/s)');
print2pdf('funcao_downwash',1);


% figure(25);
% plot(tt_ref, vi, 'b');

%% downwash

% x = 2.0:0.01:3.2;
% y = 2.0:0.01:3.2;
% z = 2.0:0.01:3.2;

[X,Y,Z] = meshgrid(-1:0.05:1,-1:0.05:1,2:0.05:3.2);
                                       %reduzir
                                       
x = reshape(X,[],1);
y = reshape(Y,[],1);
z = reshape(Z,[],1);                                       
                                       

o = 0.15;
n = 0;

% Kx = (1/(o*sqrt(2*pi)))*exp(-((x-n).^2)/(2*o.^2));
% Kx = (1/(o*sqrt(2*pi)))*exp(-((x-n).^2)/(2*o.^2));
% Ky = (1/(o*sqrt(2*pi)))*exp(-((y-n).^2)/(2*o.^2));

Kxy =(1/(o*sqrt(2*pi)))*exp(-((x-n).^2+(y-n).^2)/(2*o.^2));

Kxy(Kxy>1)=1;
% Ky(Ky>1)=1;

Kxy(Kxy<0.1) = 0;
% Ky(Ky<0.1) = 0;

% R = 0.15*0.7;
% T = R*2;
% t = -0.3:0.001:0.3;
% f = 0.5*(1 + cos(2*pi/T*(t+R))).*((t+R+T/2>0) - (t+R>0)) + ...
%          ((t+R>0)-(t-R>0)) + ...
%          0.5*(1 + cos(2*pi/T*(t-R))).*((t-R>0) - (t-R-T/2>0));
% figure(1); plot(t,f)

u = x*0;
v = y*0;

w = - vi*(1+tanh(-k*(zr-z)/h)).*Kxy;

%proppeller creation

% j = -1:0.05:1;
% k = -1:0.05:1;
% circle = (j - 3).^2 + (k - 3).^2 - R^2 + 3;
% figure(98);
% plot(j,circle);


inds = abs(w) < 0.01;

x(inds) = [];
y(inds) = [];
z(inds) = [];
u(inds) = [];
v(inds) = [];
w(inds) = []; 

figure(20);
% circle(1,1,1);
% hold on;

w = w*1000;
quiver3(x,y,z,u,v,w);

%uiver hold on, uiver hold on (ou meshgrid()

 
end

%teste do quiver3
% [X,Y,Z] = meshgrid(0:0.5:10,0:0.5:10,0:0.5:10);
% x = reshape(X,[],1);
% y = reshape(Y,[],1);
% z = reshape(Z,[],1);
% u = cos(x/5);
% v = cos(y/3);
% w = cos(z/3).^2;
% 
% inds = abs(w) < 0.1;
% 
% x(inds) = [];
% y(inds) = [];
% z(inds) = [];
% u(inds) = [];
% v(inds) = [];
% w(inds) = [];
% 
% figure(5);
% quiver3(x,y,z,u,v,w);

if scenario == 2.2
    figure(1);
    plot3(x1 ,y1 , z1, '-b');
    hold on;
    plot3(x_ref,y_ref,z2, '-r');
    grid on;
    title('Position');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis equal;
    print2pdf('position3D_2',1);
    
    figure(6);
    plot(tt_ref,x1,'-b');
    hold on;
    plot(tt_ref,x2,'-r');
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('x','x2');
    print2pdf('trajetoria_x2',1);
    
    figure(3);
    plot(tt_ref,y1,'-b');
    hold on;
    plot(tt_ref,y2,'-r');
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('y','y2');
     print2pdf('trajetoria_y2',1);
    
    figure(4);
    plot(tt_ref,z1,'-b');
    hold on;
    plot(tt_ref,z2,'-r');
    grid on;
    xlabel('Time(s)');
    ylabel('Position(m)');
    legend('z','z2');
     print2pdf('trajetoria_z2',1);
     
%      
%      
%     figure(7);
%     plot3(felted_downwash_x ,felted_downwash_y, felted_downwash_z , '-b');
%     xlabel('Time(s)');
%     ylabel('Downwash from shuttle drone (m/s)');
%     legend('dw');
%     print2pdf('downwash',1);
%      
    
end
% rotation
% figure(101);
% plot(a, '-g'); 
% xlabel('tt_ref');
% ylabel('a');
% title('Velocity');


if scenario == 4
    figure(40);
    plot(x_ref,'-b', x1, '-r');
    hold on;
    xlabel('Time(s)');
    ylabel('x');
    title('Position in X');
    
    figure(41);
    plot(y_ref, '-b', y1, '-r');
    hold on;
    xlabel('Time(s)');
    ylabel('y');
    title('Position in Y');
    
    figure(42);
    plot(z_ref, '-b', z1, '-r');
    hold on;
    xlabel('Time(s)');
    ylabel('y');
    title('Position in Z');
    
    
 
 
%     axis equal;
%     print2pdf('position3D_2',1);
    
    
end

% print2pdf('trajetoria_x2',1);

% print2pdf('trajetoria_z2',1);
% 
% 
% 
% 
% 
% 
% %angles
% for i = 1:roll(length(roll))
%     roll(i) = roll(i) * (180/pi);
% end
% figure(5);
% plot(tt_ref,roll,'-b');
% hold on;
% plot(tt_ref,roll_ref,'-g');
% xlabel('Time(s)');
% ylabel('Roll(degrees)');
% legend('roll','roll_ref');
% 
% for i = 1:yaw(length(yaw))
%     yaw(i) = yaw(i) * (180/pi);
% end
% yaw = yaw * (pi/180);
% figure(6);
% plot(tt_ref,yaw,'-b');
% hold on;
% plot(tt_ref,yaw_ref,'-g');
% xlabel('Time(s)');
% ylabel('Yaw(degrees)');
% ylim([-5*10^-3 5*10^-3]);
% legend('yaw','yaw_ref');
% 
% 
% for i = 1:pitch (length(pitch))
%     pitch(i) = pitch(i) * (180/pi);
% end
% pitch = pitch * (pi/180);
% figure(7);
% plot(tt_ref,pitch,'-b');
% hold on;
% plot(tt_ref,pitch_ref,'-g');
% xlabel('Time(s)');
% ylabel('Pitch(degrees)');
% legend('pitch','pitch_ref');
% 
% figure(8);
% plot(tt_ref,error_x,'-b');
% hold on;
% plot(tt_ref,error_y,'-g');
% hold on;
% plot (tt_ref,error_z,'-d');
% xlabel('Time(s)');
% ylabel('Error');
% legend('error_x',' error_y','error_z');
% 

% 
% Medium_error_x = 0;
% Medium_error_y = 0;
% Medium_error_z = 0;
% for i = 1:length(error_x)
%     Medium_error_x = abs(Medium_error_x) + abs(error_x(i));
%     Medium_error_y = abs(Medium_error_y) + abs(error_y(i));
%     Medium_error_z = abs(Medium_error_z) + abs(error_z(i));
% end
%  
% % Medium_error_x  = Medium_error_x/length(Medium_error_x);
% % 
% % Medium_error_y  = Medium_error_y/length(Medium_error_y);
% % 
% % Medium_error_z  = Medium_error_z/length(Medium_error_z);
% % disp('Medium errors:');
% % disp(Medium_error_x);
% % disp(Medium_error_y);
% % disp(Medium_error_z);

disp('last values');
disp(error_x(length(error_x)));
disp(error_y(length(error_y)));
disp(error_z(length(error_z)));
disp('control');
disp(u1(length(u1)));


 %% step5: Save the movie
%myWriter = VideoWriter('drone_animation', 'Motion JPEG AVI');
% myWriter = VideoWriter('drone_animation1', 'MPEG-4');
% myWriter.Quality = 100;
% myWritter.FrameRate = 120;
% 
% % Open the VideoWriter object, write the movie, and class the file
% open(myWriter);
% writeVideo(myWriter, movieVector);
% close(myWriter); 