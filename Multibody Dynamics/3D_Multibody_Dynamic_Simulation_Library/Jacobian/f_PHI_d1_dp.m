function PHI_d1_dp = f_PHI_d1_dp(Y, ai_p, bodyi, aj_p, bodyj, n, m)

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);

Ai = f_AMat(p, bodyi);
Aj = f_AMat(p, bodyj);
Gi = f_GMat(p, bodyi);

PHI_d1_dpi = -(aj_p.')*(Aj.')*Ai*f_SkewMat(ai_p);

PHI_d1_dp  = 2*PHI_d1_dpi*Gi;

end
