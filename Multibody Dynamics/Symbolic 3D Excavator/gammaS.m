function gammaS = gammaS(wi_p,siP_p,Ai,wj_p,sjP_p,Aj)
wi_pTilda = f_SkewMatrix(wi_p);
wj_pTilda = f_SkewMatrix(wj_p);

gammaS = Ai*wi_pTilda*wi_pTilda*siP_p - Aj*wj_pTilda*wj_pTilda*sjP_p;
end