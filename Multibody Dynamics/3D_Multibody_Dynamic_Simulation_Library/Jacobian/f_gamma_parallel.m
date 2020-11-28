function gamma_parallel = f_gamma_parallel(Y, fi, gi, bodyi, hj, bodyj, n, m)

gamma_d11 = f_gamma_d1(Y, fi, bodyi, hj, bodyj, n, m);
gamma_d12 = f_gamma_d1(Y, gi, bodyi, hj, bodyj, n, m);

gamma_parallel = [gamma_d11; gamma_d12];

end
