% Constructs the constraint vector from individual constraint elements

function [PHI] = Evalconstraints(Y,n,m,s)

PHI= [f_Revolute(Y, 0, s(:, 1), 1, s(:, 2), n, m); ...
      f_Revolute(Y, 1, s(:, 3), 2, s(:, 4), n, m)];
end