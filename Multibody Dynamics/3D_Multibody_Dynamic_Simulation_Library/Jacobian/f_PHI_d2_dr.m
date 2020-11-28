function PHI_d2_dr = f_PHI_d2_dr(Y, ai_p, body, n, m)

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);

A = f_AMatrix(p, body);
PHI_d2_dr = ai_p'*A';

end