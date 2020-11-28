function gammaD1 = f_gammaD1(wi_p,ai_p,Ai,wj_p,aj_p,Aj)
wi_pSkew = f_Skew(wi_p);
wj_pSkew = f_Skew(wj_p);
ai_pSkew = f_Skew(ai_p);
aj_pSkew = f_Skew(aj_p);

% Dot-1 Gamma
gammaD1 = -transpose(aj_p)*( transpose(Aj)*Ai*wi_pSkew*wi_pSkew + ...
    wj_pSkew*wj_pSkew*transpose(Aj)*Ai )*ai_p + 2*transpose(wj_p)*aj_pSkew*transpose(Aj)*Ai*ai_pSkew*wi_p;
end