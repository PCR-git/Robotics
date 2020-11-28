function PHI_spherical_dr = f_PHI_spherical_dr(body)

if body == 0
    PHI_spherical_dr = zeros(3,3);
else
    PHI_spherical_dr = eye(3,3);
end

end