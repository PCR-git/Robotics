function PHI_spherical_dp = f_PHI_spherical_dp(Y, si_p, bodyi, n, m)
[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);

s1 = si_p(1);
s2 = si_p(2);
s3 = si_p(3);

e = f_e(p, bodyi);

e0 = e(1);
e1 = e(2);
e2 = e(3);
e3 = e(4);

% PHI_spherical_dp = ...
%     [4*e0*s1 - 2*e3*s2 + 2*e2*s3   ,   4*e1*s1 + 2*e2*s2 + 2*e3*s3 , ...
%     2*e1*s2 + 2*e0*s3              ,   -2*e0*s2 + 2*e1*s3; ...
%     2*e3*s1 + 4*e0*s2 - 2*e1*s3    ,   2*e2*s1 - 2*e0*s3 , ...
%     2*e1*s1 + 4*e2*s2 + 2*e3*s3    ,   2*e0*s1 + 2*e2*s3; ...
%     -2*e2*s1 + 2*e1*s2 + 4*e0*s3   ,   2*e3*s1 + 2*e0*s2 , ...
%     -2*e0*s1 + 2*e3*s2             ,   2*e1*s1 + 2*e2*s2 + 4*e3*s3];
% Written using Mathematica

PHI_spherical_dp = 2*f_AMat(p, bodyi)*f_SkewMat(si_p)*f_GMat(p, bodyi);

end