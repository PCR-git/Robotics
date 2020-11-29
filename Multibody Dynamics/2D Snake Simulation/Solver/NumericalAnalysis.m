% Solves for the state vector Y during the specified time interval

function [T,Y,Te,Ye,PHI_T,qpp] = NumericalAnalysis(f_DAE,f_Evalconstraints,Yn,t_final,n,m,rwj,WPtol,Go)

%% Initializations
% f_DAE: handle of the DAE function
% f_Evalconstraints: handle of the function that evaluates the constraints
% Yn: initial Y state variable 
% n: number of state variables
% m: number of constraints
% t_final: final time of the simulation

%PHI_T : matrix column 1: time  column 2: norm of the position constraint at the corresponding time 
tspan = [0 t_final];

[Ypn] = f_DAE(0,Yn); % initial slope
PHI_T = [];
cpt = 1;
qpp(1, :) = Ypn(1:n)'; % initial acceleration

%% Solver
% Creates an event function for the robot, sets tolerances and
% initial slope, and handles the output with a callback function
% -----------------------------------
 opt = odeset('Event',@WayPtContact,'InitialSlope',Ypn,...
    'RelTol',1e-5,'AbsTol',1e-5,'Stats','off','OutputFcn',Callback_fct);

% Note: Relative error tolerance measures the error between two iterations,
% relative to the magnitude. AbsTol measures the error againt some total
% absolute value.

% Solves the DAE with numerical solver ode15s
%[T,Y] = ode15s(f_DAE,tspan,Yn,opt);
[T,Y,Te,Ye,~] = ode15s(f_DAE,tspan,Yn,opt);

%% ----------------------------------
% PHI_T_ode15s_DAE = PHI_T;

% function p = makeEvents(y_event)
%     % In case you want to add an event 
% p = @events;
%   
% function [value,isterminal,direction] = events(t,y)
%     val = y(4)-y_event;  
%     value = [val; val];
%     isterminal = [1; 1];            % Stop at local minimum
%     direction = [1; -1];            % [local minimum, local maximum]
% end
% end
%  ----------------------------------

%% Callback Function
% Creates vectors to hold the time and the norm of the constraints and
% appends new values to them with each iteration.
function handlefct = Callback_fct()

handlefct = @Callback_fct0;
    function status = Callback_fct0(t,y,~)
      if isempty(y)
         status = 1;
      else
         status = 0;
         if max(size(t)) > 1
            t = t(cpt);
         else
            %Y0 = y;
         end
            PHI = f_Evalconstraints(t,y);
            PHI_T(cpt,1) = t;
            PHI_T(cpt,2) = norm(PHI);  % Norm of the constraints
            yqpp = f_DAE(t,y);         % Acceleration
            qpp(cpt, :) = yqpp(1:n)';  % Appends accel to matrix
            cpt = cpt+1;
       end
    end
end

%% Event Function
% Stops simulation of snake robot when point A (head) nears Way Point

function [value,isterminal,direction] = WayPtContact(~,Y)
    
    % Track position of snake robot
     [~,~,q] = f_StateVar(Y,n,m);
     R1 = f_RM(q(3));
     v = R1*[0;1];
     A = [q(1)+v(1);q(2)+v(2)];
     
     if Go == 1
        % value = 0 when point A reaches desired position
        value = sqrt((A(1)-rwj(1))^2+(A(2)-rwj(2))^2) - WPtol;
     elseif Go == 0
        value = 0;  % Prevents event from being triggered
     end
        isterminal = 1;  % Terminate integration at event?
        direction = 0;   % 0 if all zeros are to be located
end

end