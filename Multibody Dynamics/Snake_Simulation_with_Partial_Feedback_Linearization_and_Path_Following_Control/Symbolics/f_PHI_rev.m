
function PHI_rev = f_PHI_rev(s1P,s2P,s3P,s4P,s5P,s6P,s7P,s8P,s9P,s10P)

syms x1 y1 phi1 x2 y2 phi2 x3 y3 phi3...
     x4 y4 phi4 x5 y5 phi5 x6 y6 phi6

q = [x1;y1;phi1;x2;y2;phi2;x3;y3;phi3;...
     x4;y4;phi4;x5;y5;phi5;x6;y6;phi6];

r1 = [q(1);q(2)];
r2 = [q(4);q(5)];
r3 = [q(7);q(8)];
r4 = [q(10);q(11)];
r5 = [q(13);q(14)];
r6 = [q(16);q(17)];

A1 = f_RM(q(3));
A2 = f_RM(q(6));
A3 = f_RM(q(9));
A4 = f_RM(q(12));
A5 = f_RM(q(15));
A6 = f_RM(q(18));

Rev1 = r1+A1*s1P-r2-A2*s2P;
Rev2 = r2+A2*s3P-r3-A3*s4P;
Rev3 = r3+A3*s5P-r4-A4*s6P;
Rev4 = r4+A4*s7P-r5-A5*s8P;
Rev5 = r5+A5*s9P-r6-A6*s10P;

PHI_rev = [Rev1; Rev2; Rev3; Rev4; Rev5]

PHIq_rev = jacobian(PHI_rev,q)

end
