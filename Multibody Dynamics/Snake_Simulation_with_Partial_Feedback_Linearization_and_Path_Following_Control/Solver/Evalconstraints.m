% Constructs the constraint vector

function [PHI] = Evalconstraints(Y,n,m,s)

% Local position vectors
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

% Constraint vector (each row corresponds to a constraint)
PHI= [f_Revolute(Y,1,sA1,2,sA2,n,m);...
      f_Revolute(Y,2,sB2,3,sB3,n,m);...
      f_Revolute(Y,3,sC3,4,sC4,n,m);...
      f_Revolute(Y,4,sD4,5,sD5,n,m);...
      f_Revolute(Y,5,sE5,6,sE6,n,m)];
  
end