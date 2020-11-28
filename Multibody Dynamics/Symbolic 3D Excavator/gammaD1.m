function gammaD1 = gammaD1(wi_p,ai_p,Ai,wj_p,aj_p,Aj)
wi_pTilda = f_SkewMatrix(wi_p);
wj_pTilda = f_SkewMatrix(wj_p);
ai_pTilda = f_SkewMatrix(ai_p);
aj_pTilda = f_SkewMatrix(aj_p);

gammaD1 = -transpose(aj_p)*( transpose(Aj)*Ai*wi_pTilda*wi_pTilda + ...
    wj_pTilda*wj_pTilda*transpose(Aj)*Ai )*ai_p + 2*transpose(wj_p)*aj_pTilda*transpose(Aj)*Ai*ai_pTilda*wi_p;
end