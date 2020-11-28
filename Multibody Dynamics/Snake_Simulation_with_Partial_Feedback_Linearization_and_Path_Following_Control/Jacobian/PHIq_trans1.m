% Element of the Jacobian, corresponding to 1st translational constraint

function tA1 = PHIq_trans1(q,body1,V1prime)

tA1 = transpose(V1prime)*transpose(f_B(f_angle(q,body1)));

end