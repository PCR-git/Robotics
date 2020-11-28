function gammaS = f_gammaS(wi,siP,Ai,wj,sjP,Aj)
wiSkew = f_Skew(wi);
wjSkew = f_Skew(wj);

% Spherical Gamma
gammaS = Ai*wiSkew*wiSkew*siP - Aj*wjSkew*wjSkew*sjP;

end