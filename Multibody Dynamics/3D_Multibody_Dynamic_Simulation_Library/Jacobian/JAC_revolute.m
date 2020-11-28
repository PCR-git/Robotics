function [JAC_revolute_dr, Jac_revolute_dp, Jac_revolute_p_dp] = JAC_revolute(Y, si_p, fi, gi, bodyi, sj_p, hj, bodyj, n, m)

%%  JacRev r
if bodyi == 0
    PHI_revolute_dr2 =  zeros(5,3);
    PHI_revolute_dr1 =  f_PHI_revolute_dr(bodyj);
else
    PHI_revolute_dr1 =  -f_PHI_revolute_dr(bodyi);
    PHI_revolute_dr2 = f_PHI_revolute_dr(bodyj);
end    

JAC_revolute_dr = [PHI_revolute_dr1 PHI_revolute_dr2];

%%  JacRev p
if bodyi == 0
    PHI_revolute_dp1 = f_PHI_revolute_dpj(Y, si_p, fi, gi, bodyi, sj_p, hj, bodyj, n, m);
    PHI_revolute_dp2 = zeros(5,4);
else
    PHI_revolute_dp1 = f_PHI_revolute_dp(Y, si_p, fi, gi, bodyi, hj, bodyj, n, m);
    PHI_revolute_dp2 = f_PHI_revolute_dpj(Y, si_p, fi, gi, bodyi, sj_p, hj, bodyj, n, m);
end


Jac_revolute_dp = [PHI_revolute_dp1 PHI_revolute_dp2];

%%  JacRev pp
% f_PHI_p_dp1 = f_PHI_p_dp(Y, bodyi, n, m);
% f_PHI_p_dp2 = f_PHI_p_dp(Y, bodyj, n, m);
% 
% Jac_revolute_p_dp = [f_PHI_p_dp1 f_PHI_p_dp2];

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);
Jac_revolute_p_dp = 2*blkdiag((p(1:4).'), (p(5:8).'));

end