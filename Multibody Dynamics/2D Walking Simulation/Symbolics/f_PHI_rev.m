% Computes symbolic Jacobian vector (called in BuildMechanism)

function PHI_rev = f_PHI_rev(sA1,sA2,sB2,sB3,sC3,sC4,sD4,sD5,sE5,sE6,sF6,sF7,sG7,sG8)

syms x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 x4 y4 phi4...
     x5 y5 phi5 x6 y6 phi6 x7 y7 phi7 x8 y8 phi8

q = [x1;y1;phi1;x2;y2;phi2;x3;y3;phi3;x4;y4;phi4;...
     x5;y5;phi5;x6;y6;phi6;x7;y7;phi7;x8;y8;phi8];

r1 = [q(1);q(2)];
r2 = [q(4);q(5)];
r3 = [q(7);q(8)];
r4 = [q(10);q(11)];
r5 = [q(13);q(14)];
r6 = [q(16);q(17)];
r7 = [q(19);q(20)];
r8 = [q(22);q(23)];

A1 = f_RM(q(3));
A2 = f_RM(q(6));
A3 = f_RM(q(9));
A4 = f_RM(q(12));
A5 = f_RM(q(15));
A6 = f_RM(q(18));
A7 = f_RM(q(21));
A8 = f_RM(q(24));

Rev1 = r1+A1*sA1-r2-A2*sA2;
Rev2 = r2+A2*sB2-r3-A3*sB3;
Rev3 = r3+A3*sC3-r4-A4*sC4;
Rev4 = r4+A4*sD4-r5-A5*sD5;
Rev5 = r5+A5*sE5-r6-A6*sE6;
Rev6 = r6+A6*sF6-r7-A7*sF7;
Rev7 = r7+A7*sG7-r8-A8*sG8;

PHI_rev = [Rev1; Rev2; Rev3; Rev4; Rev5; Rev6; Rev7];

PHIq_rev = jacobian(PHI_rev,q)

end
