
syms phi1(t) phi2(t) phi3(t) ...
     lambda1 lambda2 lambda3 lambda4 ...
     I1 I2 I3 ...
     u1 u2
 
Dphi1 = diff(phi1);
phi1(t) = dsolve(diff(I1*phi1,t,t) + lambda1*phi1+lambda2 + 0.5*u1 == 0, phi1(0) == 0, Dphi1(0) == 0);

Dphi2 = diff(phi2);
phi2(t) = dsolve(diff(I2*phi2,t,t) + (lambda1+lambda3)*1+(lambda2+lambda4)*phi2 - 0.5*u1 + 0.5*u2 == 0, phi2(0) == 0, Dphi2(0) == 0);

Dphi3 = diff(phi3);
phi3(t) = dsolve(diff(I3*phi3,t,t) + lambda3*1+lambda4*phi3 - 0.5*u2 == 0, phi3(0) == 0, Dphi3(0) == 0);

phi1(t)
phi2(t)
phi3(t)