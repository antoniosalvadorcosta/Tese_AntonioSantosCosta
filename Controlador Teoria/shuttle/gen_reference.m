function [p_d,v_d,a_d,j_d,psi_d,dpsi_d] = gen_reference(t,iD,P)

    if iD == 1
        p0 = P.("p0_" + num2str(1));
        psi0 = P.("psi0_" + num2str(1));
    else
        p0 = P.("p0_" + num2str(2));
        psi0 = P.("psi0_" + num2str(2));
    end
%     eval(['p0 = P.p0_' num2str(iD) ';']);
%     eval(['psi0 = P.psi0_' num2str(iD) ';']);

    if P.ref_mode == 1 % square wave reference
        t_step = 10;
        sw = mod(t,t_step)>=t_step/2; % boolean wave
        p_d = p0 + P.p_ref_static*sw;
        v_d = zeros(3,1);
        a_d = zeros(3,1);
        j_d = zeros(3,1);
        psi_d = psi0 + P.psi_ref_static*sw;
        dpsi_d = 0;
        
    else % circle reference
        phase = (iD-1)*P.dphase;
        p_d = [P.Rad*cos(P.omn*t+phase);P.Rad*sin(P.omn*t+phase);P.vz_d*t+P.p_ref_static(3)];
        v_d = [-P.Rad*P.omn*sin(P.omn*t+phase);P.Rad*P.omn*cos(P.omn*t+phase);P.vz_d];
        a_d = [-P.Rad*P.omn^2*cos(P.omn*t+phase);-P.Rad*P.omn^2*sin(P.omn*t+phase);0];
        j_d = [ P.Rad*P.omn^3*sin(P.omn*t+phase);-P.Rad*P.omn^3*cos(P.omn*t+phase);0];
        psi_d = atan2(v_d(2),v_d(1));
        dpsi_d = P.omn;
    end

end

