function [f1,f2,f3,f4] = control_allocation(fc, T_x, T_y, T_z, alpha)
    % Constants (adjust as needed)
    kt = 1; % Example value for thrust coefficient
    kq = 1; % Example value for torque coefficient
    
    % Control allocation matrix G
    G = [1, 1, 1, -1;
         -kt*sin(alpha), kt*sin(alpha), kt*sin(alpha), -kt*sin(alpha);
         -kt*cos(alpha), kt*cos(alpha), -kt*cos(alpha), kt*cos(alpha);
         -kq/kt, -kq/kt, kq/kt, kq/kt];
    
   
    F = [fc; T_x; T_y; T_z];
    
    % Calculate control outputs
     controls = F./G;
    
    f1 = controls(1);   
    f2 = controls(2);  
    f3 = controls(3);  
    f4 = controls(4);  
end
