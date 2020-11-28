function gammaSSd = gammaSSd(ri,dri,wi,siP_p,Ai,rj,drj,wj,sjP_p,Aj,s1,t)
wiTilda = f_SkewMatrix(wi);
wjTilda = f_SkewMatrix(wj);
siP_pTilda = f_SkewMatrix(siP_p);
sjP_pTilda = f_SkewMatrix(sjP_p);

dij = rj + Aj*sjP_p - ri - Ai*siP_p;

gammaSS = -2*transpose(drj - dri)*(drj - dri) + 2*transpose(sjP_p)*wjTilda*wjTilda*sjP_p + ...
    2*transpose(2*siP_p)*wiTilda*wiTilda*siP_p - 4*transpose(sjP_p)*wjTilda*transpose(Aj)*Ai*wiTilda*siP_p + ...
    4*transpose(drj - dri)*( Aj*sjP_pTilda*wj - Ai*siP_pTilda*wi ) - ...
    2*transpose(dij)*( Ai*wiTilda*siP_pTilda*wi - Aj*wjTilda*sjP_pTilda*wj );

gammaSSd = gammaSS + 2*s1*(diff(diff(s1,t),t)) + 2*diff(s1,t)*diff(s1,t);
end