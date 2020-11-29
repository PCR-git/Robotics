% Element of the Jacobian, corresponding to 3rd translational constraint

function tA3 = PHIq_trans3(q,body1,body2,V1prime,V2prime)

tA3 = transpose(V1prime)*transpose(f_RM(f_angle(q,body1)))*f_RM(f_angle(q,body2))*V2prime;

end