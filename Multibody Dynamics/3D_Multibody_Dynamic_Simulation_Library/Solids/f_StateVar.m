function [r_dt, p_dt, lambda, lambda_p, r, p] = f_StateVar(Y, n, m)

nbd=n/7;
r_dt     = Y(1:(3*nbd), 1);
p_dt     = Y((3*nbd + 1):n, 1);

lambda   = Y((n + 1):(n + m - nbd), 1);
lambda_p = Y((n + m - nbd + 1):(n + m), 1);

r        = Y((n + m + 1):(n + m + 3*nbd), 1);
p        = Y((n + m + 3*nbd + 1):(n + m + n), 1);

end
