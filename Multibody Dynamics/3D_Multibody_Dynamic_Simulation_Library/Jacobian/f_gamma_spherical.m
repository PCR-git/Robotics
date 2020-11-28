function gamma_spherical = f_gamma_spherical(Y, si_p, bodyi, sj_p, bodyj, n, m)

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);
if bodyi == 0
Aj = f_AMat(p, bodyj);
w_pj = f_w_p(Y, bodyj, n, m);
w_p_tj = f_SkewMat(w_pj);
else
Ai = f_AMat(p, bodyi);
Aj = f_AMat(p, bodyj);
w_pi = f_w_p(Y, bodyi, n, m);
w_pj = f_w_p(Y, bodyj, n, m);
w_p_ti = f_SkewMat(w_pi);
w_p_tj = f_SkewMat(w_pj);    
end

if bodyi == 0
    gamma_spherical = - Aj*w_p_tj*w_p_tj*sj_p;
else
    gamma_spherical = Ai*w_p_ti*w_p_ti*si_p - Aj*w_p_tj*w_p_tj*sj_p;
end

end
