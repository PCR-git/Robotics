function PHI_revolute_dpj = f_PHI_revolute_dpj(Y, si_p, fi, gi, bodyi, sj_p, hj, bodyj, n, m)

PHI_spherical_dpj = f_PHI_spherical_dpj(Y, sj_p, bodyj, n, m);
PHI_parallel_dpj  = f_PHI_parallel_dpj(Y, fi, gi, bodyi, hj, bodyj, n, m);

PHI_revolute_dpj = [PHI_spherical_dpj; PHI_parallel_dpj];

end
