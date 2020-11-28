function gamma_p = f_gamma_p(Y, body, n, m)

[~, p_dt, ~, ~, ~, ~] = f_StateVar(Y, n, m);
p_dt_b = f_e(p_dt, body);
gamma_p = -2*(p_dt_b.')*p_dt_b;

end
