% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% Summary: simulate simple dynamic system model of a drone

% Initialize arrays and variables for downwash values
global vd_store_2;
global v_air_store;



v_air_store = [0;0];
vd_store_2 = [0;0];
dw = 0;
tracking_sim = 0;
v_air = [0;0;0];


% main time loop for simulation
for k = 1:Param.Nsim
    for iD = 1:Param.nD
                
        % get state vector
        p_2{iD} = x_2{iD}(1:3,k);
        v_2{iD} = x_2{iD}(4:6,k);
        R = reshape(x_2{iD}(7:15,k),3,3);
        om= x_2{iD}(16:18,k);
        

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
            other_pos{iD} = p_2{other_id};
            other_thrust{iD} = T_2{other_id}(:,k);
            other_vel{iD} = v_2{other_id};
        
        else
            other_pos{iD} = [0;0;0];
            other_thrust{iD} = 0;
            other_vel{iD} = [0;0;0]; 
        end
        
        
        % mellinger controller
        [T_2{iD}(:,k),tau_2{iD}(:,k),e_p] = drone_mellinger_ctrl_cpte(p_2{iD},v_2{iD},R,om,Param,p_d,psi_d,xiep{iD}(:,k),v_d,dpsi_d,a_d,j_d, dw_comp,other_pos{iD},other_thrust{iD});
        
        
        % control allocation
        [f1,f2,f3,f4] = control_allocation(T_2{iD}(1,k), tau_2{iD}(1,k), tau_2{iD}(2,k), tau_2{iD}(3,k), Param.alpha); 
        F = [f1,f2,f3,f4];
        
        % integrate position error
        xiep{iD}(:,k+1) = xiep{iD}(:,k) + Param.dTi*e_p;
        

        % nonlinear drone model (continuous time)
        [dot_p,dot_v,dot_R,dot_om] = drone_3dfull_dyn_cpte(p_2{iD},v_2{iD},R,om,T_2{iD}(:,k),tau_2{iD}(:,k),Param,other_pos{iD},other_thrust{iD});
        
    
        % discretization 
        pp = p_2{iD} + Param.dTi*dot_p; 
        vp = v_2{iD} + Param.dTi*dot_v;
        Rp = rot_integrate(R,om,Param.dTi);
        omp = om + Param.dTi*dot_om;
        x_2{iD}(:,k+1) = [pp;vp;reshape(Rp,[],1);omp];

        
        % auxiliary drone attitude computation from rotation matrix
        lbd_2{iD}(:,k) = R2Euler(R);

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
%clear p;
for iD = 1:Param.nD
    x_2{iD}(:,k+1) = [];
    p_2{iD} = x_2{iD}(1:3,:);
    % Store final vd for this drone
    
end

x{3} = x_2{2};
p{3} = p_2{2};
v{3} = v_2{2};
T{3} = T_2{2};
lbd{3} = lbd_2{2};
tau{3} = tau_2{2};
