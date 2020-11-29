% Specifies the Jacobian PHIq, Gamma (from the acceleration equation),
% and the force vector Qa

function [M,PHIq,Gamma]=BuildMechanism(Y,n,m,M,s)

% Local Position Vectors
sA1 = s(:,3);
sA2 = s(:,4);
sB2 = s(:,5);
sB3 = s(:,6);

% % Constructs the constraint vector (symbolically)
% % (Pass into FeedForward symbolic function)
% PHI_rev = f_PHI_rev(sA1,sA2,sB2,sB3);

% Constructs the Jacobian (each row corresponds to a constraint) 
PHIq = [JAC_revolute(Y,1,sA1,2,sA2,n,m);...
        JAC_revolute(Y,2,sB2,3,sB3,n,m)];

% Constructs Gamma (each row corresponds to a constraint)    
Gamma = [Gamma_revolute(Y, 1, sA1, 2, sA2, n, m);...
         Gamma_revolute(Y, 2, sB2, 3, sB3, n, m)];

end