% Solves for the state vector Y during the specified time interval

function [T,Y,PHI_T,qpp] = NumericalAnalysis(f_DAE,f_Evalconstraints,Yn,t_final,n,~)

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
% opt = odeset('Event',@groundContact,'InitialSlope',Ypn,...
%    'RelTol',1e-7,'AbsTol',1e-8,'Stats','on','OutputFcn',Callback_fct);
opt = odeset('InitialSlope',Ypn,'RelTol',10e-8,...
             'AbsTol',10e-8,'Stats','on','OutputFcn',Callback_fct);

% Note: Relative error tolerance measures the error between two iterations,
% relative to the magnitude. AbsTol measures the error againt some total
% absolute value.

% Solves the DAE with numerical solver ode15s
%[T,Y,~,~,~] = ode15s(f_DAE,tspan,Yn,opt);
[T,Y] = ode15s(f_DAE,tspan,Yn,opt);

%% ----Integration Experiment----
% body1 = 1; body2 = 2; body3 = 3;
% nb = n/3;
% q = Y(:, n+nb*2+1:n+nb*2+n);
% Integral = cumtrapz(T,q(:,3+3*(body3-1)));
% figure
% plot(T, q(:,3+3*(body3-1)),'g');
% hold on;
% plot(T, Integral,'b');
% -------------------------------

%% ------
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
%    end
% end

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
% % Stops simulation of snake robot when
% % point C reaches -L/3 below 'ground' frame
% function [value,isterminal,direction] = groundContact(~,Y)
%     % track position for bipedal robot
%      [~,~,q] = f_StateVar(Y,n,m);
%      C = f_cg(q,2)+f_RM(f_angle(q,2))*[0;-1]; % position of point C
%      value = C(2)+1/3; % zero when point C reaches desired position
%      isterminal = 1;
%      direction = 0;
% end

end