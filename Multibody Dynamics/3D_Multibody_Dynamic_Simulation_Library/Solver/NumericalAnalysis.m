function [T, Y, PHI_T, qpp] = NumericalAnalysis(f_DAE, f_Evalconstraints, Yn, t_final, n, m)

% Initialization
tspan       = [0 t_final];                  % time vector
PHI         = f_Evalconstraints(0, Yn);     % initial PHI
PHI_T = (PHI);                    % initial norm of PHI
Ypn         = f_DAE(0, Yn);                 % initial state vector derivative
qpp(1, :)   = Ypn(1:n)';                    % initial acceleration
ii          = 2;                      

% ODE solver
% Creates an event function, sets tolerances and
% initial slope, and handles the output with a callback function
opt = odeset('InitialSlope', Ypn,...        % initial slope of ODE (qpp)
                     'RelTol', 1e-7, ...    % relative error tolerance (max)
                     'AbsTol', 1e-8, ...    % absolute error tolerance (min)
                     'Stats', 'on', ...     % solver statistics
                     'OutputFcn', ODE_fh,...% output function handle
                     'Events', @impact);    % event function
[T, Y] = ode15s(f_DAE, tspan, Yn, opt);% numerical ODE solver
% Relative error tolerance measures the error between two iterations,
% relative to the magnitude. AbsTol measures the error againt some total
% absolute value.

% function p = makeEvents(y_event)
%     % In case you want to add an event 
% p = @events;
%   
% function [value,isterminal,direction] = events(t,y)
%     val=y(4)-y_event;  
%     value = [val; val];
%     isterminal = [1; 1];            % Stop at local minimum
%     direction = [1; -1];            % [local minimum, local maximum]
%    end
% end

% output function handle
function handlefct = ODE_fh()             
handlefct = @ODE_fh0;                   
    function status = ODE_fh0(t, y, flag)  
        if isempty(y)                      
            status = 1;                  
        else                         
            status = 0;                    
            if max(size(t)) > 1            
            else                            
                PHI = f_Evalconstraints(t,y);
                PHI_T(:,ii) = norm(PHI);  
                yqpp = f_DAE(t,y);          
                qpp(ii, :) = yqpp(1:n)';  
                ii = ii + 1;               
            end                     
        end                               
    end                                 
end                                    

% Stops simulation  when point C reaches -L/3 below ground frame
function [value, isterminal, direction] = impact(T, Y)  % event function
%     q = Y((n + m + 1):end);                 
%     ep = f_r(q, 2, [0; -1]); 
%     value = ep(2) + 0.3;             
    value = 1;
    isterminal = 1;                         
    direction  = 0;                         
end                                         

end
