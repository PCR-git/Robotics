
function PHI_trans = f_PHI_trans(Sprime1,Sprime2,V1prime,V2prime)

syms q1x q1y q1phi q2x q2y q2phi

q = [q1x;q1y;q1phi;q2x;q2y;q2phi];

r1 = [q(1);q(2)];
r2 = [q(4);q(5)];

R = [0,-1;1,0];

tB1 = transpose(V1prime)*transpose(f_B(q(3)))*(r2-r1)...
     -transpose(V1prime)*f_B(q(6)-q(3))*Sprime2...
     -transpose(V1prime)*transpose(R)*Sprime1;

tB2 = -transpose(V1prime)*f_B(q(6)-q(3))*V2prime;

PHI_trans = [tB1;tB2];

end