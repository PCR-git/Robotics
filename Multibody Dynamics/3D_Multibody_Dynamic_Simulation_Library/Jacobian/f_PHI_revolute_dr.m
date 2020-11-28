function PHI_revolute_dr = f_PHI_revolute_dr(body)

PHI_spherical_dr = f_PHI_spherical_dr(body);
PHI_parallel_dr  = f_PHI_parallel_dr();

PHI_revolute_dr = [PHI_spherical_dr; PHI_parallel_dr];


end
