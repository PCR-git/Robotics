function PHI_parallel = f_PHI_parallel(Y, Pi, Gi, bodyi, Hj, bodyj, n, m)

PHI_parallel = [f_PHI_d1(Y, Pi, bodyi, Hj, bodyj, n, m); ...
                f_PHI_d1(Y, Gi, bodyi, Hj, bodyj, n, m)];

end
