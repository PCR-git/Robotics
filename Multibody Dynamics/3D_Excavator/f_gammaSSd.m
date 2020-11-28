function gammaSSd = f_gammaSSd(ri,dri,wi,siP,Ai,rj,drj,wj,sjP,Aj,s1,t)
wiSkew = f_Skew(wi);
wjSkew = f_Skew(wj);
siPSkew = f_Skew(siP);
sjPSkew = f_Skew(sjP);

dij = rj + Aj*sjP - ri - Ai*siP;

% Spherical-Spherical Gamma
gammaSS = -2*transpose(drj - dri)*(drj - dri) + 2*transpose(sjP)*wjSkew*wjSkew*sjP + ...
    2*transpose(siP)*wiSkew*wiSkew*siP - 4*transpose(sjP)*wjSkew*transpose(Aj)*Ai*wiSkew*siP + ...
    4*transpose(drj - dri)*( Aj*sjPSkew*wj - Ai*siPSkew*wi ) - ...
    2*transpose(dij)*( Ai*wiSkew*siPSkew*wi - Aj*wjSkew*sjPSkew*wj );

gammaSSd = gammaSS + 2*s1*(diff(diff(s1,t),t)) + 2*diff(s1,t)*diff(s1,t);
end