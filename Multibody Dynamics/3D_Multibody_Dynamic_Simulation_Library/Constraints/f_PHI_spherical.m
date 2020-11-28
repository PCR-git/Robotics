function PHI_spherical = f_PHI_spherical(Y, si_p, bodyi, sj_p, bodyj, n, m)

[~, ~, ~, ~, r, p] = f_StateVar(Y, n, m);

if bodyi ~= 0 && bodyi ~= 0        
    [rAi] = f_r(r, p, bodyi, si_p);   
    [rAj] = f_r(r, p, bodyj, sj_p);    
else                            
    if bodyi == 0          
        [rAi] = [0; 0; 0];
        [rAj] = f_r(r, p, bodyj, sj_p);
    else                           
        error('wrong configuration')
    end                           
end                             

PHI_spherical = rAi - rAj;               

end
