function JAC_Rev_Symbolic

syms L1 L2 L3 ...
     m1 I1 m2 I2 m3 I3 ...
     x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 ...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p ...
     x1pp y1pp phi1pp x2pp y2pp phi2pp x3pp y3pp phi3pp ...
     lambda1 lambda2 lambda3 lambda4 ...
     u1 u2 ...
     u1b u2b ...
     f1 f2 f3 f4 f5 f6 f7 f8 f9

nb = 3;         % Number of bodies
n = nb*3;       % Number of generalized coordinates
%m = (nb-1)*2;   % Number of constraints 

body1 = 1;
body2 = 2;
body3 = 3;

B1 = [-sin(phi1),-cos(phi1); cos(phi1),-sin(phi1)];
B2 = [-sin(phi2),-cos(phi2); cos(phi2),-sin(phi2)];
B3 = [-sin(phi3),-cos(phi3); cos(phi3),-sin(phi3)];

sA1 = [0; -L1];   % Point A from frame 1
sA2 = [0;  L2];   % Point A from frame 2
sB2 = [0; -L2];   % Point B from frame 2
sB3 = [0;  L3];   % Point B from frame 3

JAC_rev = zeros(2,n);

PHIq_rev1 = B1*sA1;
PHIq_rev2 = B2*sA2;

JAC_rev(:,(3*(body1-1)+1):(3*body1)) = [+eye(2), PHIq_rev1];
JAC_rev(:,(3*(body2-1)+1):(3*body2)) = [-eye(2), PHIq_rev2];

JAC3 = B3*sB2;


