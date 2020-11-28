% Specifies the Jacobian PHIq, Gamma (from the acceleration equation),
% and the force vector Qa

function [M,PHIq,Gamma] = BuildMechanism(Y,n,m,M,s)

% Local Position Vectors
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

% % Constructs the constraint vector (symbolically)
% % (Pass into FeedForward symbolic function)
% PHI_rev = f_PHI_rev(sA1,sA2,sB2,sB3,sC3,sC4,sD4,sD5,sE5,sE6);

% Constructs the Jacobian (each row corresponds to a constraint) 
PHIq = [JAC_revolute(Y,1,sA1,2,sA2,n,m);...
        JAC_revolute(Y,2,sB2,3,sB3,n,m);...
        JAC_revolute(Y,3,sC3,4,sC4,n,m);...
        JAC_revolute(Y,4,sD4,5,sD5,n,m);...
        JAC_revolute(Y,5,sE5,6,sE6,n,m)];

% Constructs Gamma (each row corresponds to a constraint)    
Gamma = [Gamma_revolute(Y,1,sA1,2,sA2,n,m);...
         Gamma_revolute(Y,2,sB2,3,sB3,n,m);...
         Gamma_revolute(Y,3,sC3,4,sC4,n,m);...
         Gamma_revolute(Y,4,sD4,5,sD5,n,m);...
         Gamma_revolute(Y,5,sE5,6,sE6,n,m)];

end