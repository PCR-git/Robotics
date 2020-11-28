function PHI_revolute_dp = f_PHI_revolute_dp(Y, si_p, fi, gi, bodyi, hj, bodyj, n, m)

PHI_spherical_dp = f_PHI_spherical_dp(Y, si_p, bodyi, n, m);
PHI_parallel_dp  = f_PHI_parallel_dp(Y, fi, gi, bodyi, hj, bodyj, n, m);

PHI_revolute_dp = [PHI_spherical_dp; PHI_parallel_dp];

end
