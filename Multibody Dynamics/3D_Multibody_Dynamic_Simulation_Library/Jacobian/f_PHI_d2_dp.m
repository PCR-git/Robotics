function PHI_d2_dp = f_PHI_d2_dp(Y, si_p, ai_p, bodyi, sj_p, bodyj, n, m)

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);

Ai = f_AMatrix(p, bodyi);
dij = f_r(r, p, bodyi, si_p) - f_r(r, p, bodyj, sj_p);
Gi = f_GMatrix(p, bodyi);

PHI_d2_dpi = ai_p'*f_SkewMatrix(si_p) - dij'*Ai*f_SkewMatrix(ai_p);
PHI_d2_dp = 2*PHI_d2_dpi*Gi;

end
