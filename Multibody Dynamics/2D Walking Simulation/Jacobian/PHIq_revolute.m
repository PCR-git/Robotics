% Element of the Jacobian, corresponding to revolute constraint

function rA = PHIq_revolute(q,body,Sprime)

rA = f_B(f_angle(q,body))*Sprime;

end