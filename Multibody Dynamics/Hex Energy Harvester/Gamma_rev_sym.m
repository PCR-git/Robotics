function Gamma_rev = Gamma_rev_sym

syms L1 L2 L3 ...
     x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 ...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p

sA1 = [0; -L1];   % Point A from frame 1
sA2 = [0;  L2];   % Point A from frame 2
sB2 = [0; -L2];   % Point B from frame 2
sB3 = [0;  L3];   % Point B from frame 3

R1 = f_RM(phi1);
R2 = f_RM(phi2);
R3 = f_RM(phi3);

gamma1 = R1*sA1*phi1p^2 - R2*sA2*phi2p^2;

gamma2 = R2*sB2*phi2p^2 - R3*sB3*phi3p^2;

gamma = [gamma1; gamma2]