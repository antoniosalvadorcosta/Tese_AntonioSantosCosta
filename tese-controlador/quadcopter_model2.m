function [d_p,d_v,d_R,d_W]= quadcopter_model2(p,v,a,R,w,u,Parameter,aux,m,time)
%QUADROTOR_MODEL Summary of this function goes here

dx = 0.50;
dy = 0.236;
dz = 0;
D = diag ([dx, dy, dz]);

P = Parameter;
c = u(1)/P.m + P.kh*(v'*(R(:,1)+R(:,2))).^2;
% estava    "+"


%Pitt and Peters
%Pc =  (15*pi/32)*tan(x/2)

if P.wind == 1
    if P.rd_model == 0
%         if u(1) < 0
%             disp('Drone 2 unstable!');
%             disp(u(1));
%             
%             %  vi = sqrt(-u(1)/(2*P.air_d*P.A));
%             
%         else
%             vi = sqrt(u(1)/(2*P.air_d*P.A));
%         end
%         disp('vi drone:');
%         disp(vi);
%         
%         
%         aux = v-Vw-vi;
        
        
        
        d_p = v;
        d_v = -[0;0;P.g] + 1/P.m*R*[0;0;u(1)]- (1/2)*P.air_d*P.Cd*P.Pa*((aux(1:3,1)).^2).*sign(aux(1:3,1));
        d_W = P.I^-1*(-(cross(w,P.I*w))+u(2:4));
        d_R = R*skew(w);
        
      
        
    else % With rotor drag (maior diferença)
        
        
        %     d_p = v;
        %     d_v = -[0;0;P.g] + 1/m*R*[0;0;u(1)] - R*D*R'*v;
        %     d_W =  P.I^-1*(-(cross(w,P.I*w))+u(2:4));
        %     d_R = R*skew(w);
%         %
%         d_p = v;
%         d_v = -[0;0;P.g] + c*R(:,3) - R*D*R'*v;
%         d_W =  P.I^-1*(-(cross(w,P.I*w))+u(2:4));
%         d_R = R*skew(w);
%         %
   
    %the drone catches another one
   
  
        % else
        %  "Influence of Aerodynamics and Proximity Effects in Quadrotor Flight"
        % Determinar experimentalmente
        % T = k1*(rs^2)+k2*(V+v)*rs;
        
        d_p = v;           % 1/P.m*R*[0;0;u(1)]
        d_v = -[0;0;P.g] + c*R(:,3) - R*D*R'*v -(1/2)*P.air_d*P.Cd*P.Pa*((aux(1:3,1)).^2).*sign(aux(1:3,1));
        d_W =  P.I^-1*(-(cross(w,P.I*w))+u(2:4));
        d_R = R*skew(w);
        
     end 

else 
    if P.rd_model == 0
        d_p = v;
        d_v = -[0;0;P.g] + 1/P.m*R*[0;0;u(1)];
        d_W = P.I^-1*(-(cross(w,P.I*w))+u(2:4));
        d_R = R*skew(w);
        
        
        
    else % With rotor drag (maior diferença)
        
        
        %     d_p = v;
        %     d_v = -[0;0;P.g] + 1/m*R*[0;0;u(1)] - R*D*R'*v;
        %     d_W =  P.I^-1*(-(cross(w,P.I*w))+u(2:4));
        %     d_R = R*skew(w);
        %
        d_p = v;
        d_v = -[0;0;P.g] + c*R(:,3) - R*D*R'*v;
        d_W =  P.I^-1*(-(cross(w,P.I*w))+u(2:4));
        d_R = R*skew(w);
        %
    end  

end

%- 0.1*v.*abs(v); 