function Vc = f_dw(p1,p2, T, P)

            Vc = [];
            % downwash shapping parameters sugested by Gemini
            k = 0.8;                % between 0 and 1
            h = 2*P.rotor_radius;          % in the range of rotor diameter or slightly larger


          
            for i = 1:P.Nsim
                % height differenc4e between rotor and a point bellow
                height_diff = p1(3,i) - p2(3,i);
                
                % induced velocity
                vi = sqrt(T(i)./(2*P.air_d*P.A));
                
                
                % radial distance between rotor center and the downwash
                % boundary (Momentum theory)
                dw_radius = sqrt( (T(i)) ./ (pi * P.air_d  * vi.^2) );
                
                condition =  P.rotor_radius ./ sqrt(1 + tanh(-k*( height_diff )./h));
                
                if dw_radius < condition
                    % downwash vertical velocity
                    vc = vi + vi * tanh(-k * (height_diff) / h);
                else
                    vc = 0 ; % Add 0 if condition not met
                end
                disp(vc);
                Vc = [Vc,vc];
                
            end
end

