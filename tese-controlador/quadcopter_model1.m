function [d_p,wc,d_v,d_R,d_W]= quadcopter_model(p,v,a,R,w,u,Parameter,Vw,m,time,vi_2)
%QUADROTOR_MODEL Summary of this function goes here

dx = 0.50;
dy = 0.236;
dz = 0;
D = diag ([dx, dy, dz]);

P = Parameter;
c = u(1)/P.m + P.kh*(v'*(R(:,1)+R(:,2))).^2;
% estava    "+"


%vi_2 the produced downwash from the quadrotor bellow

thrust_relation = 0;
%Pitt and Peters
%Pc =  (15*pi/32)*tan(x/2)
vi = 0;

disp(vi_2);
wc = 0;
%vi_2 = 0;
if P.wind_2 == 1
    if P.rd_model_2 == 0
        
        
        if u(1) < 0
            disp('Drone 2 unstable!');
            disp(u(1));
            
            %  vi = sqrt(-u(1)/(2*P.air_d*P.A));
            
        else
            vi = sqrt(u(1)/(2*P.air_d*P.A));
           if 0.5 < P.rotor_radius/(sqrt(1+tanh(-5*((p(3)-3)/1))))
                wc = vi+vi*tanh(-5*((p(3)-3)/1));
           else
                wc = 0;
           end
        end
        
        disp ('distance between drones:')
        disp(p(3)-3);
        disp('wc drone 2:');
        disp(wc);
        
        if P.scenario == 2
            aux = v-Vw-(wc);
        end
        aux = v-Vw-wc;
        
        
        
        d_p = v;
        d_v = -[0;0;P.g] + 1/P.m*R*[0;0;u(1)] -(1/2)*P.air_d*P.Cd*P.Pa*((aux(1:3,1)).^2).*sign(aux(1:3,1));
        d_W = P.I^-1*(-(cross(w,P.I*w))+u(2:4));
        d_R = R*skew(w);
        
        
        
    else % With rotor drag (maior diferenša)
        
        
        if u(1) < 0
            disp('Drone 2 unstable!');
            disp(u(1));
            
            %  vi = sqrt(-u(1)/(2*P.air_d*P.A));
            
        else
            vi = sqrt(u(1)/(2*P.air_d*P.A));
            wc = vi+vi*tanh(-3*((p(3)-3)/5));
        end
        disp ('distance between drones:')
        disp(p(3)-3);
        disp('wc drone 2:');
        disp(wc);
        
        if P.scenario == 2
            aux = v-Vw-(wc);
        end
        aux = v-Vw-wc;
        
        
    
        
        %the drone catches another one
        
        
        %     %Calculate induced velocity
        
        %         disp('Considering wind on drone 2');
        
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
    if P.rd_model_2 == 0
        d_p = v;
        d_v = -[0;0;P.g] + 1/P.m*R*[0;0;u(1)];
        d_W = P.I^-1*(-(cross(w,P.I*w))+u(2:4));
        d_R = R*skew(w);
        
        
        
    else % With rotor drag (maior diferenša)
        
        
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