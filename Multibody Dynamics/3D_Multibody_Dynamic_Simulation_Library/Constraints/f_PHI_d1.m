function PHI_d1 = f_PHI_d1(Y, ai_p, bodyi, aj_p, bodyj, n, m)

[~, ~, ~, ~, ~, p] = f_StateVar(Y, n, m);

if bodyi ~= 0 && bodyj ~= 0        
    PHI_d1 = ai_p'*(f_AMat(p, bodyi))'*f_AMat(p, bodyj)*aj_p;
else                               
    if bodyi == 0                  
        PHI_d1 = ai_p'*f_AMat(p, bodyj)*aj_p;
    else                           
        error('wrong configuration')
    end                            
end 


