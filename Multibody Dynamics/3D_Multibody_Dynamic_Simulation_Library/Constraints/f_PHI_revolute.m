function PHI_revolute = f_PHI_revolute(Y, si_p, Pi, Gi, bodyi, sj_p, Hj, bodyj, n, m)

PHI_spherical = f_PHI_spherical(Y, si_p, bodyi, sj_p, bodyj, n, m);
PHI_parallel  = f_PHI_parallel(Y, Pi, Gi, bodyi, Hj, bodyj, n, m);

PHI_revolute = [PHI_spherical; PHI_parallel];

end
