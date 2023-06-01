function animation = anima(x1,y1,z1,roll1,pitch1,yaw1,x2,y2,z2)
% This Animation code is for QuadCopter.   
 

%% 1. define the motion coordinates 
 

%     z_ref  = 4*tt_ref;
%     y_ref  = 3*tt_ref;
%     x_ref  = 2*tt_ref;
% 
% disp(x);
% disp(x_ref);

D2R = pi/180;
R2D = 180/pi;
b   = 1;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
      sin(ro) cos(ro)  0;
       0       0       1];     % rotation matrix to rotate the coordinates of base 
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
           -a/2 -a/2 a/2 a/2;
             0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree 
to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
%% Define Figure plot
 fig1 = figure('pos', [0 50 800 600]);
 hg   = gca;
 view(68,53);
 grid on;
 axis equal;
 xlim([-5 35]); ylim([-10 10]); zlim([0 10]);
 title('Drone Animation')
 xlabel('X[m]');
 ylabel('Y[m]');
 zlabel('Z[m]');
 hold(gca, 'on');
 
%% Design Different parts
% design the base square
 drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
 drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
 alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
 [xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
 drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
 drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
 alpha(drone(3:4),0.6);
% design 4 cylindrical motors 
 drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
 drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
 alpha(drone(5:8),0.7);
% design 4 propellers
 drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'c','LineWidth',0.5);
 alpha(drone(9:12),0.3);
 
 %drone2--------------------------------------------------------------------
 drone2(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
 drone2(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
 alpha(drone2(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
 [xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
 drone2(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
 drone2(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
 alpha(drone2(3:4),0.6);
% design 4 cylindrical motors 
 drone2(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone2(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone2(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
 drone2(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
 alpha(drone2(5:8),0.7);
% design 4 propellers
 drone2(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'b','LineWidth',0.5);
 drone2(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'b','LineWidth',0.5);
 drone2(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'b','LineWidth',0.5);
 drone2(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'b','LineWidth',0.5);
 alpha(drone(9:12),0.3);
 
 
%% create a group object and parent surface


combinedobject = hgtransform('parent',hg );
set(drone,'parent',combinedobject)
pause(0.2);

combinedobject2 = hgtransform('parent',hg );
set(drone2,'parent',combinedobject2)
pause(0.2);
%plot3(x_ref,y_ref, z_ref, '-b');
 
 
  
%  drawnow
 
%  for i = 1:length(z_ref)
%   
%      ba = plot3(x_ref(1:i),y_ref(1:i),z_ref(1:i), 'g:','LineWidth',1.5);
%  
%    
%      translation_ref = makehgtform('translate',...
%                                [x_ref(i) y_ref(i) z_ref(i)]);
%  
%      set(combinedobject, 'matrix',translation_ref);
%    
% %      rotation1_ref = makehgtform('xrotate',(pi/180)*(roll_ref(i)));
% %      rotation2_ref  = makehgtform('yrotate',(pi/180)*(pitch_ref(i)));
% %      rotation3_ref  = makehgtform('zrotate',yaw_ref(i));
%      
%      %scaling = makehgtform('scale',1-i/20);
%      
%        %set(combinedobject,'matrix',...
%         % translation_ref*rotation3_ref*rotation2_ref*rotation1_ref );
%       %movieVector(i) =  getframe(fig1);
%         %delete(b);
%      drawnow

%  end
 for i = 1:length(z1)
  
     
     ba_ref = plot3(x1(1:i),y1(1:i),z1(1:i), 'b','LineWidth',1.5);
     
  
     hold on;
     
     ba_ref = plot3(x2(1:i),y2(1:i),z2(1:i), 'r','LineWidth',1.5);
     
     translation  = makehgtform('translate',...
                               [x1(i) y1(i) z1(i)]);  
                  
     translation2 = makehgtform('translate',...
                               [x2(i) y2(i) z2(i)]); 
                           
                           
     %set(combinedobject, 'matrix',translation);
     rotation1  = makehgtform('xrotate',(pi/180)*(roll1(i)));
     rotation2  = makehgtform('yrotate',(pi/180)*(pitch1(i)));
     rotation3  = makehgtform('zrotate', yaw1(i));
      
     
     %scaling = makehgtform('scale',1-i/20);
     set(combinedobject,'matrix',...
          translation*rotation3*rotation2 *rotation1 );
    
     set(combinedobject2,'matrix',...
          translation2);
      %movieVector(i) =  getframe(fig1);
        %delete(b);
     drawnow
    %pause(0.2);
 end
 
