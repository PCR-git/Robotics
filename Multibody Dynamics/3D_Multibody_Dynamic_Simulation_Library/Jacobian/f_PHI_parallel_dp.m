function PHI_parallel_dp = f_PHI_parallel_dp(Y, fi, gi, bodyi, hj, bodyj, n, m)

PHI_d1_dp1 = f_PHI_d1_dp(Y, fi, bodyi, hj, bodyj, n, m);
PHI_d1_dp2 = f_PHI_d1_dp(Y, gi, bodyi, hj, bodyj, n, m);

PHI_parallel_dp = [PHI_d1_dp1; PHI_d1_dp2];

end
