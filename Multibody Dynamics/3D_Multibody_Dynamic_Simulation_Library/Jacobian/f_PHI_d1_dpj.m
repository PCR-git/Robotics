function PHI_d1_dpj = f_PHI_d1_dpj(Y, ai_p, bodyi, aj_p, bodyj, n, m)

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);

Ai = f_AMat(p, bodyi);
Aj = f_AMat(p, bodyj);
Gj = f_GMat(p, bodyj);

PHI_d1_dpi = -(ai_p.')*(Ai.')*Aj*f_SkewMat(aj_p);
PHI_d1_dpj  = 2*PHI_d1_dpi*Gj;

end
