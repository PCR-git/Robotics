function w_p = f_w_p(Y, body, n, m)

[~, p_dt, ~, ~, ~, p] = f_StateVar(Y, n, m);

p_dt_body = f_e(p_dt, body);
w_p = 2*f_GMat(p, body)*p_dt_body;

end
