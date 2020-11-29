% Element of the Jacobian, corresponding to 2nd translational constraint

function tA2 = PHIq_trans2(q,body1,body2,Sprime2,V1prime,r1,r2)

tA2 = transpose(V1prime)*transpose(f_RM(f_angle(q,body1)))*(r2-r1)+...
      transpose(V1prime)*transpose(f_RM(f_angle(q,body1)))*f_RM(f_angle(q,body2))*Sprime2;

end