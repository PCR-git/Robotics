% Constructs the elements of the Jacobian due to translational constraints

function JAC_trans = JAC_translational(Y,body1,body2,Sprime2,V1prime,V2prime,n,m)

JAC_trans = zeros(2,n);
[~,~,q] = f_StateVar(Y,n,m);

if body1 ~= 0 && body2 ~= 0
    
    r1 = q(3*body1-2:3*body1-1);
    r2 = q(3*body2-2:3*body2-1);
    
    JAC_trans(:,(3*(body1-1)+1):(3*body1)) = [-PHIq_trans1(q,body1,V1prime), -PHIq_trans2(q,body1,body2,Sprime2,V1prime,r1,r2);...
                                                                 zeros(1,2), -PHIq_trans3(q,body1,body2,V1prime,V2prime)];
    JAC_trans(:,(3*(body2-1)+1):(3*body2)) = [+PHIq_trans1(q,body1,V1prime), +PHIq_trans4(q,body1,body2,Sprime2,V1prime);...
                                                                 zeros(1,2), +PHIq_trans3(q,body1,body2,V1prime,V2prime)];
else
    if body1 == 0
    JAC_trans(:,(3*(body2-1)+1):(3*body2)) = [+PHIq_trans1(q,body1,V1prime), +PHIq_trans4(q,body1,body2,Sprime2,V1prime);...
                                                                 zeros(1,2), +PHIq_trans3(q,body1,body2,V1prime,V2prime)];
    else
        error('wrong configuration')
    end
end