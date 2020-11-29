function qp = InitV(v0,s,n,m,q)

% z=zeros(n+m,1);
% Y=[z;q];

Y=zeros(2*n+m,1);
Y(n+m+1:end)=q;

PHIq= [JAC_revolute(Y,0,s(:,1),1,s(:,2),n,m);...
       JAC_revolute(Y,1,s(:,3),2,s(:,4),n,m)];
   
% Determines the null space of the Jacobian
Re0=null(PHIq); %dimension n*DOF

qp_dof=[v0];% Desired velocity for the DOF
index_DOF=[3 6];% index if the DOF is related to the q state variable
%ex: q=[x1 y1 phi1 x2 y2 phi2]'  phi1 and phi2 are my DOF variables

alpha=eye(2)/Re0(index_DOF,:); % Determines alpha such that it rewrites the null space Re0
% to a desired null space Re that will conserve the velocity of the DOF when determining 
%qp=Re*qp_dof.

Re=Re0*alpha;

qp=Re*qp_dof;

%check 
PHIqp=PHIq*qp;

PHIqp=round(10^12*PHIqp)/10^12; %precision at 10^12 is enough to avoid eps error

end
