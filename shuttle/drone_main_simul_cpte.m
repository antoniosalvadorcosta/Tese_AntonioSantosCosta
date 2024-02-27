% Project Capture
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

% Summary: simulate simple dynamic system model of a drone

% Initialize arrays for downwash values
vd_store= [];
dw = 0;

global pos_d2;
global vi_d2;

vi_d2 = 0;
pos_d2 = Param.p2;

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

%         if k == 1 || iD == 2
%             dw = 0;
%         end
        
        % mellinger controller
        [T{iD}(:,k),tau{iD}(:,k),e_p] = drone_mellinger_ctrl_cpte(p{iD},v{iD},R,om,Param,p_d,psi_d,xiep{iD}(:,k),v_d,dpsi_d,a_d,j_d,iD,dw);
        
        
        % integrate position error
        xiep{iD}(:,k+1) = xiep{iD}(:,k) + Param.dTi*e_p;
        
        
        % downwash
        
        
        if iD == 2
            dw =  f_dw3( p{1} , p{2}, T{1}(:,k),T{2}(:,k), Param);
            vd_store = [vd_store; dw];
        end
        
        
        
        % nonlinear drone model (continuous time)
        [dot_p,dot_v,dot_R,dot_om] = drone_3dfull_dyn_cpte(v{iD},R,om,T{iD}(:,k),tau{iD}(:,k),Param,iD,dw);
        
        
        
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
    % Store final vd for this drone
    
end



