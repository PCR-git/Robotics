function gamma_d1 = f_gamma_d1(Y, ai_p, bodyi, aj_p, bodyj, n, m)

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);

Ai = f_AMat(p, bodyi);
Aj = f_AMat(p, bodyj);
w_pi = f_w_p(Y, bodyi, n, m);
w_pj = f_w_p(Y, bodyj, n, m);
w_p_ti = f_SkewMat(w_pi);
w_p_tj = f_SkewMat(w_pj);
ai_p_t = f_SkewMat(ai_p);
aj_p_t = f_SkewMat(aj_p);

gamma_d1 = -(aj_p.')*((Aj.')*Ai*w_p_ti*w_p_ti + w_p_tj*w_p_tj*(Aj.')*Ai)*ai_p + ...
            2*(w_pj.')*aj_p_t*(Aj.')*Ai*ai_p_t*w_pi;

end
