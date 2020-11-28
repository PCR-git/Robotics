function PHI_p_dp = f_PHI_p_dp(Y, body, n, m)

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);

Gi = f_GMatrix(p, body);
e = f_e(p, body);

PHI_p_dpi = e'*Gi';
PHI_p_dp = 2*PHI_p_dpi*Gi;

end
