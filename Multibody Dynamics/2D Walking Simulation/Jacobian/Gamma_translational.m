% Returns the elements of Gamma due to translational constraints

function Gamma_trans = Gamma_translational(Y, body1, body2, V1prime, Sprime2, n, m)

%Gamma_trans = zeros(2, 1);   
[qp, ~, q] = f_StateVar(Y, n, m);

if body1 ~= 0 && body2 ~= 0

    r1 = q(3*body1-2:3*body1-1);
    r2 = q(3*body2-2:3*body2-1);

    r1dot = qp(3*body1-2:3*body1-1);
    r2dot = qp(3*body2-2:3*body2-1);

    phi1dot = qp(3*body1);
    phi2dot = qp(3*body2);

    Gamma_trans = -[transpose(V1prime)*(transpose(f_B(f_angle(q,body1)))*f_B(f_angle(q,body2))*Sprime2*(phi2dot-phi1dot)^2 -...
                   transpose(f_B(f_angle(q,body1)))*(r2-r1)*phi1dot^2 - 2*transpose(f_RM(f_angle(q,body1)))*(r2dot-r1dot)*phi1dot);...
                                                                                                                                 0];

else
    if body1 == 0
        
        phi2dot = qp(3*body2);

        Gamma_trans = -[transpose(V1prime)*transpose(f_B(0))*f_B(f_angle(q,body2))*Sprime2*phi2dot^2; 0];
    else
       error('wrong configuration'); 
    end
end

end