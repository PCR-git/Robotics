function QA_sym = f_QA_sym

syms x1 y1 phi1 x2 y2 phi2 x3 y3 phi3 ...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p ...
     u1 u2

R1 = f_RM(phi1);
R2 = f_RM(phi2);
R3 = f_RM(phi3);

R1T = transpose(R1);
R2T = transpose(R2);
R3T = transpose(R3);

%q = [x1;y1;phi1;x2;y2;phi2;x3;y3;phi3];
qp = [x1p;y1p;phi1p;x2p;y2p;phi2p;x3p;y3p;phi3p];
 
T1 = -0.5*u1;
T2 = 0.5*u1 - 0.5*u2;
T3 = 0.5*u2;

T = [0;0;T1;...
     0;0;T2;...
     0;0;T3];

% ----------------------------

rlp1 = R1T*[qp(1);qp(2)];   % Velocity vector of center of upper link
rlp2 = R2T*[qp(4);qp(5)];   % Velocity vector of center of lower link
rlp3 = R3T*[qp(7);qp(8)];   % Velocity vector of center of lower link

xlp1 = rlp1(1);
xlp2 = rlp2(1);
xlp3 = rlp3(1);
 
% Friction in both directions
Fv1 = [xlp1; 0];
Fv2 = [xlp2; 0];
Fv3 = [xlp3; 0];

% Angular friction
Fa1 = 0;
Fa2 = 0;
Fa3 = 0;

% Total Friction Vector
Ff = [R1*Fv1; Fa1; R2*Fv2; Fa2; R3*Fv3; Fa3];
   
% ----------------------------

Q = Ff + T

end