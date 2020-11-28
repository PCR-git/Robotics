function PHI_d2_dpj = f_PHI_d2_dpj(Y, si_p, ai_p, bodyi, sj_p, bodyj, n, m)

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);

Ai = f_AMatrix(p, bodyi);
Aj = f_AMatrix(p, bodyj);
Gj = f_GMatrix(p, bodyj);

PHI_d2_dpji = ai_p'*Ai*Aj*f_SkewMatrix(sj_p);
PHI_d2_dpj = 2*PHI_d2_dpji*Gj;

end
