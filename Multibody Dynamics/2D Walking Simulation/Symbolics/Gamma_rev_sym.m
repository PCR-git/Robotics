% Computes symbolic Gamma vector

function Gamma_rev = Gamma_rev_sym

syms L1 L2 L3 L4... 
     L5 L6 L7 L8...
     x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 x4 y4 phi4...
     x5 y5 phi5 x6 y6 phi6 x7 y7 phi7 x8 y8 phi8...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p x4p y4p phi4p...
     x5p y5p phi5p x6p y6p phi6p x7p y7p phi7p x8p yp8 phi8p...

sA1 = [0; -L1];
sA2 = [0;  L2];
sB2 = [0; -L2];
sB3 = [0;  L3];
sC3 = [0; -L3];
sC4 = [0;  L4];
sD4 = [0; -L4];
sD5 = [0;  L5];
sE5 = [0; -L5];
sE6 = [0;  L6];
sF6 = [0; -L6];
sF7 = [0;  L7];
sG7 = [0; -L7];
sG8 = [0;  L8];

R1 = f_RM(phi1);
R2 = f_RM(phi2);
R3 = f_RM(phi3);
R4 = f_RM(phi4);
R5 = f_RM(phi5);
R6 = f_RM(phi6);
R7 = f_RM(phi7);
R8 = f_RM(phi8);

gamma1 = R1*sA1*phi1p^2 - R2*sA2*phi2p^2;
gamma2 = R2*sB2*phi2p^2 - R3*sB3*phi3p^2;
gamma3 = R3*sC3*phi3p^2 - R4*sC4*phi4p^2;
gamma4 = R4*sD4*phi4p^2 - R5*sD5*phi5p^2;
gamma5 = R5*sE5*phi5p^2 - R6*sE6*phi6p^2;
gamma6 = R6*sF6*phi6p^2 - R7*sF7*phi7p^2;
gamma7 = R7*sG7*phi7p^2 - R8*sG8*phi8p^2;

gamma = [gamma1; gamma2; gamma3; gamma4; gamma5; gamma6; gamma7]

