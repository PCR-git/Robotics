function PHI_rev = f_PHI_rev(s1P,s2P,s3P,s4P)

syms x1 y1 phi1 x2 y2 phi2 x3 y3 phi3

q = [x1;y1;phi1;x2;y2;phi2;x3;y3;phi3];

r1 = [q(1);q(2)];
r2 = [q(4);q(5)];
r3 = [q(7);q(8)];

A1 = f_RM(q(3));
A2 = f_RM(q(6));
A3 = f_RM(q(9));

Rev1 = r1+A1*s1P-r2-A2*s2P;

Rev2 = r2+A2*s3P-r3-A3*s4P;

PHI_rev = [Rev1; Rev2]

PHIq_rev = jacobian(PHI_rev,q)

end