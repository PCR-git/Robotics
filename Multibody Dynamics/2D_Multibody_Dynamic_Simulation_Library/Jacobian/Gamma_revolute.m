% Returns the elements of Gamma due to revolute constraints

function Gamma_rev = Gamma_revolute(Y, body1, Sprime1, body2, Sprime2, n, m)

Gamma_rev  = zeros(2, 1);   
[qp, ~, q] = f_StateVar(Y, n, m);

if body1 ~= 0 && body2 ~= 0
   Gamma_rev = f_RM(f_angle(q, body1))*Sprime1*qp(3*body1)^2 ...
             - f_RM(f_angle(q, body2))*Sprime2*qp(3*body2)^2;
else
    if body1 == 0
        Gamma_rev = - f_RM(f_angle(q, body2))*Sprime2*qp(3*body2)^2;
    else
       error('wrong configuration'); 
    end
end

end