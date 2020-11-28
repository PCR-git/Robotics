function QA_sym_local = f_QA_sym_local

syms x1 yl phi1 x2 y2 phi2 x3 y3 phi3 ...
     x1lp y1lp phi1lp x2lp y2lp phi2lp x3lp y3lp phi3lp ...
     u1 u2

R1 = f_RM(phi1);
R2 = f_RM(phi2);
R3 = f_RM(phi3);

R1T = transpose(R1);
R2T = transpose(R2);
R3T = transpose(R3);

%q = [x1;y1;phi1;x2;y2;phi2;x3;y3;phi3];
qlp = [x1lp;y1lp;phi1lp;x2lp;y2lp;phi2lp;x3lp;y3lp;phi3lp];

T1 = -0.5*u1;
T2 = 0.5*u1 - 0.5*u2;
T3 = 0.5*u2;

T = [0;0;T1;...
     0;0;T2;...
     0;0;T3];

% ----------------------------

% rllp = [qlp(1);qlp(2)];   % Velocity vector of center of upper link
% r2lp = [qlp(4);qlp(5)];   % Velocity vector of center of lower link
% r3lp = [qlp(7);qlp(8)];   % Velocity vector of center of lower link
% 
% xllp = r1lp(1);
% xllp = r2lp(1);
% xllp = r3lp(1);
 
% Friction in both directions
Fv1 = [x1lp; 0];
Fv2 = [x2lp; 0];
Fv3 = [x3lp; 0];

% Angular friction
Fa1 = 0;
Fa2 = 0;
Fa3 = 0;

% Total Friction Vector
Ff = [R1*Fv1; Fa1; R2*Fv2; Fa2; R3*Fv3; Fa3];
   
% ----------------------------

Q = Ff + T

end