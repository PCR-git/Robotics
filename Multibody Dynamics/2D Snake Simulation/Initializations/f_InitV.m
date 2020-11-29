% Given the initial velocity of the degrees of freedom, this function sets
% the total initial velocity vector of the robot while maintaining the
% consistency of the constraints

function qp = f_InitV(v0,s,n,m,q)

Y = zeros(2*n+m,1);
Y(n+m+1:end) = q;

% The Jacobian of the system
PHIq = [JAC_revolute(Y,0,s(:,1),1,s(:,2),n,m);...
        JAC_revolute(Y,1,s(:,3),2,s(:,4),n,m);...
        JAC_revolute(Y,2,s(:,5),3,s(:,6),n,m)];
   
% Determines the null space of the Jacobian
Re0 = null(PHIq); % dimension n*DOF

qp_dof = v0;            % Desired velocity for the DOF
index_DOF = [3 6 9];    % index if the DOF is related to the q state variable
% ex: q = [x1 y1 phi1 x2 y2 phi2]'  phi1 and phi2 are my DOF variables

% Determines alpha such that it rewrites the null space Re0 to a desired
% null space Re that will conserve the velocity of the DOF when determining 
% qp = Re*qp_dof.
alpha = eye(3)/Re0(index_DOF,:);

Re = Re0*alpha;

qp = Re*qp_dof;

% % Check 
% PHIqp = PHIq*qp;
% 
% PHIqp = round(10^12*PHIqp)/10^12; %precision at 10^12 is enough to avoid eps error

end
