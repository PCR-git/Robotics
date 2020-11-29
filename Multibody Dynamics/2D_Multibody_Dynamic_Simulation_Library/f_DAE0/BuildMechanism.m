% Specifies the Jacobian PHIq, Gamma (from the acceleration equation),
% and the force vector Qa

function [M,PHIq,Gamma,Qa]=BuildMechanism(Y,n,m,M,s)

qp = Y(1:n);
q = Y((n+m+1):end);

% Constructs the Jacobian from the elements due to each constraint
PHIq= [JAC_revolute(Y,0,s(:,1),1,s(:,2),n,m);...
       JAC_revolute(Y,1,s(:,3),2,s(:,4),n,m)];

% Constructs Gamma from the elements due to each constraint
Gamma = [Gamma_revolute(Y, 0, s(:, 1), 1, s(:, 2), n, m); ...
        Gamma_revolute(Y, 1, s(:, 3), 2, s(:, 4), n, m)];

g=9.8;
% Force Vector
Qa = [0;-M(2,2)*g;0;0;-M(5,5)*g;0];

end