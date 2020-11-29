% Given the initial velocity of the degrees of freedom, this function sets
% the total initial velocity vector of the robot while maintaining the
% consistency of the constraints

function qp = f_InitV(v0,s,n,m)

nb = n/3;
Y = zeros(2*n+m,1);
%q = Y(n+m+1:end);

sO0 = s(:,1);
sO1 = s(:,2);
sA1 = s(:,3);
sA2 = s(:,4);
sB2 = s(:,5);
sB3 = s(:,6);
sC3 = s(:,7);
sC4 = s(:,8);
sD4 = s(:,9);
sD5 = s(:,10);
sE5 = s(:,11);
sE6 = s(:,12);
sF6 = s(:,13);
sF7 = s(:,14);
sG7 = s(:,15);
sG8 = s(:,16);
     
PHIqd = [ 0, 0, -1, 0, 0, 2, 0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0;...
          0, 0,  0, 0, 0, 0, 0, 0, -1, 0, 0, 2, 0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0;...
          0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, -1, 0, 0, 2, 0, 0,  0, 0, 0, 0;...
          0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, -1, 0, 0, 2];
% The Jacobian of the system
PHIq = [JAC_revolute(Y,0,sO0,1,sO1,n,m);...
        JAC_revolute(Y,1,sA1,2,sA2,n,m);...
        JAC_revolute(Y,2,sB2,3,sB3,n,m);...
        JAC_revolute(Y,3,sC3,4,sC4,n,m);...
        JAC_revolute(Y,4,sD4,5,sD5,n,m);...
        JAC_revolute(Y,5,sE5,6,sE6,n,m);...
        JAC_revolute(Y,6,sF6,7,sF7,n,m);...
        JAC_revolute(Y,7,sG7,8,sG8,n,m);...
        PHIqd];
   
% Determines the null space of the Jacobian
Re0 = null(PHIq); % dimension n*DOF

qp_dof = v0;         % Desired velocity for the DOF
index_DOF = 3:6:n;   % index if the DOF is related to the q state variable
% ex: q = [x1 y1 phi1 x2 y2 phi2]'  phi1 and phi2 are my DOF variables

% Determines alpha such that it rewrites the null space Re0 to a desired
% null space Re that will conserve the velocity of the DOF when determining 
% qp = Re*qp_dof.
[a,~] = size(PHIqd);
alpha = eye(nb-a)/Re0(index_DOF,:);

Re = Re0*alpha;

qp = Re*qp_dof;

% % Check 
% PHIqp = PHIq*qp;
% 
% PHIqp = round(10^12*PHIqp)/10^12; %precision at 10^12 is enough to avoid eps error

end
