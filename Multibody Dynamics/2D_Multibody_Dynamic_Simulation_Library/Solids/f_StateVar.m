% Returns the position and velocity vectors and Lagrange multiplier values
% from the state vector Y

function [qp, lambda, q] = f_StateVar(Y,n,m)

qp=Y(1:n);
lambda=Y((n+1):(n+m));
q=Y((n+m+1):end);

end