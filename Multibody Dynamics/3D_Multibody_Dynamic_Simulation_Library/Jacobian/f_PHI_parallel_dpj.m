function PHI_parallel_dpj = f_PHI_parallel_dpj(Y, fi, gi, bodyi, hj, bodyj, n, m)

PHI_d1_dpj1 = f_PHI_d1_dpj(Y, fi, bodyi, hj, bodyj, n, m);
PHI_d1_dpj2 = f_PHI_d1_dpj(Y, gi, bodyi, hj, bodyj, n, m);

PHI_parallel_dpj = [PHI_d1_dpj1; PHI_d1_dpj2];

end
