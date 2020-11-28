function PHI_spherical_dpj = f_PHI_spherical_dpj(Y, sj_p, bodyj, n, m)
[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);

% s1 = sj_p(1);
% s2 = sj_p(2);
% s3 = sj_p(3);
% 
% e = f_e(p, bodyj);
% 
% e0 = e(1);
% e1 = e(2);
% e2 = e(3);
% e3 = e(4);

% PHI_spherical_dpj = ...
%     [-4*e0*s1 + 2*e3*s2 - 2*e2*s3   ,  -4*e1*s1 - 2*e2*s2 - 2*e3*s3 , ...
%     -2*e1*s2 - 2*e0*s3              ,  2*e0*s2 - 2*e1*s3; ...
%     -2*e3*s1 - 4*e0*s2 + 2*e1*s3    ,  -2*e2*s1 + 2*e0*s3, ...
%     -2*e1*s1 - 4*e2*s2 - 2*e3*s3    ,  -2*e0*s1 - 2*e2*s3; ...
%     2*e2*s1 - 2*e1*s2 - 4*e0*s3     ,  -2*e3*s1 - 2*e0*s2, ...
%     2*e0*s1 - 2*e3*s2               ,  -2*e1*s1 - 2*e2*s2 - 4*e3*s3];
% Written using Mathematica

PHI_spherical_dpj = -2*f_AMat(p, bodyj)*f_SkewMat(sj_p)*f_GMat(p, bodyj);

end