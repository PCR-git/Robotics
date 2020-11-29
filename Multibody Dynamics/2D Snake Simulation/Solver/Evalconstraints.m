% Constructs the constraint vector

function [PHI] = Evalconstraints(Y,n,m,s)

% Local position vectors
sA1 = s(:,3);
sA2 = s(:,4);
sB2 = s(:,5);
sB3 = s(:,6);

% Constraint vector (each row corresponds to a constraint)
PHI= [f_Revolute(Y,1,sA1,2,sA2,n,m);...
      f_Revolute(Y,2,sB2,3,sB3,n,m)];
  
end