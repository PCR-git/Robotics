function PHI_d2 = f_PHI_d2(Y, ai_p, si_p, bodyi, sj_p, bodyj, n, m)

[~, ~, ~, ~, r, p] = f_StateVar(Y, n, m);

PHI_d2 = ai_p'*(f_AMatrix(p, bodyi))'* ...
            (f_r(r, p, bodyj, sj_p) - f_cg(q , bodyj)) - ai_p'*si_p';
            

end
