% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% Summary: simulate simple dynamic system model of a drone

% Initialize arrays and variables for downwash values
global vd_store;
global v_air_store;

vd_store = [0;0];

v_air_store = zeros(3,2);

dw = 0;
tracking_sim = 0;
v_air = [0;0;0];


% main time loop for simulation
for k = 1:Param.Nsim
    for iD = 1:Param.nD
                
        % get state vector
        p{iD} = x{iD}(1:3,k);
        v{iD} = x{iD}(4:6,k);
        R = reshape(x{iD}(7:15,k),3,3);
        om = x{iD}(16:18,k);
        
        % get reference
        p_d = p_ref{iD}(:,k);
        v_d = v_ref{iD}(:,k);
        a_d = a_ref{iD}(:,k);
        j_d = j_ref{iD}(:,k);
        psi_d = psi_ref{iD}(:,k);
        dpsi_d = dpsi_ref{iD}(:,k);
        
        
        
        % Store position and thrust values from the other drone
        if k>2
            other_id = 3 - iD; % Get the index of the other drone
            other_pos{iD} = p{other_id};
            other_thrust{iD} = T{other_id}(:,k);
            other_vel{iD} = v{other_id};
        
        else
            other_pos{iD} = [0;0;0];
            other_thrust{iD} = 0;
            other_vel{iD} = [0;0;0]; 
        end
        
        
        % mellinger controller
        [T{iD}(:,k),tau{iD}(:,k),e_p] = drone_mellinger_ctrl_cpte(p{iD},v{iD},R,om,Param,p_d,psi_d,xiep{iD}(:,k),v_d,dpsi_d,a_d,j_d, dw_comp,other_pos{iD},other_thrust{iD});
        
        
        % control allocation
        [f1,f2,f3,f4] = control_allocation(T{iD}(1,k), tau{iD}(1,k), tau{iD}(2,k), tau{iD}(3,k), Param.alpha); 
        F = [f1,f2,f3,f4];
        
        % integrate position error
        xiep{iD}(:,k+1) = xiep{iD}(:,k) + Param.dTi*e_p;
        

        
        % nonlinear drone model (continuous time)
        [dot_p,dot_v,dot_R,dot_om] = drone_3dfull_dyn_cpte(p{iD},v{iD},R,om,T{iD}(:,k),tau{iD}(:,k),Param,other_pos{iD},other_thrust{iD});
        
    
        % discretization 
        pp = p{iD} + Param.dTi*dot_p; 
        vp = v{iD} + Param.dTi*dot_v;
        Rp = rot_integrate(R,om,Param.dTi);
        omp = om + Param.dTi*dot_om;
        x{iD}(:,k+1) = [pp;vp;reshape(Rp,[],1);omp];
        
        

        % auxiliary drone attitude computation from rotation matrix
        lbd{iD}(:,k) = R2Euler(R);

        if abs(1-norm(Rp'*Rp))>1e-4
            warning(['Problems with rotation matrix integration... stoping simulation: t = ' num2str(k*dTi) ' s.']);
            break;
        end 

    end
   
    if abs(1-norm(Rp'*Rp))>1e-4
        break;
    
    end
end


% trim vectors to same length
clear p;
for iD = 1:Param.nD
    x{iD}(:,k+1) = [];
    p{iD} = x{iD}(1:3,:);

end



