% Constructs the elements of the Jacobian due to revolute constraints

function JAC_rev = JAC_revolute(Y,body1,Sprime1,body2,Sprime2,n,m)

JAC_rev = zeros(2,n);
[~,~,q] = f_StateVar(Y,n,m);

if body1 ~= 0 && body2 ~= 0
    PHIq_rev1 = +PHIq_revolute(q,body1,Sprime1);
    PHIq_rev2 = -PHIq_revolute(q,body2,Sprime2);
    JAC_rev(:,(3*(body1-1)+1):(3*body1)) = [+eye(2), PHIq_rev1];
    JAC_rev(:,(3*(body2-1)+1):(3*body2)) = [-eye(2), PHIq_rev2];
else
    if body1 == 0
        PHIq_rev2 = -PHIq_revolute(q,body2,Sprime2);
        JAC_rev(:,(3*(body2-1)+1):(3*body2)) = [-eye(2), PHIq_rev2];
    else
        error('wrong configuration')
    end
end