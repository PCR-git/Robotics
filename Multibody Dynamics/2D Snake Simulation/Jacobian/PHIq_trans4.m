% Element of the Jacobian, corresponding to 4th translational constraint

function tA4 = PHIq_trans4(q,body1,body2,Sprime2,V1prime)

tA4 = transpose(V1prime)*transpose(f_RM(f_angle(q,body1)))*f_RM(f_angle(q,body2))*Sprime2;

end