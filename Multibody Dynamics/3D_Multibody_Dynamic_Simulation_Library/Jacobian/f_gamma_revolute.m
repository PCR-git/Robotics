function gamma_revolute = f_gamma_revolute(Y, si_p, fi, gi, bodyi, sj_p, hj, bodyj, n, m)

gamma_spherical = f_gamma_spherical(Y, si_p, bodyi, sj_p, bodyj, n, m);
gamma_parallel = f_gamma_parallel(Y, fi, gi, bodyi, hj, bodyj, n, m);

gamma_revolute = [gamma_spherical; gamma_parallel];

end
