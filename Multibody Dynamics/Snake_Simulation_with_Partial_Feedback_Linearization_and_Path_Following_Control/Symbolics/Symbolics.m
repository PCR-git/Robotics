% Symbolic expressions for determining feedforward
% control input Ui, in f_DAE0

% Input values from Gamma_rev_sym and f_PHI_rev
% for Gamma & PHIq, respectively
% (the latter is located inside BuildMechanism)

function Symbolics

close all;
clear all;
clc;

syms L1 L2 L3...
     L4 L5 L6...
     m1 I1 m2 I2 m3 I3...
     m4 I4 m5 I5 m6 I6...
     x1 y1 phi1 x2 y2 phi2 x3 y3 phi3...
     x4 y4 phi4 x5 y5 phi5 x6 y6 phi6...
     x1p y1p phi1p x2p y2p phi2p x3p y3p phi3p...
     x4p y4p phi4p x5p y5p phi5p x6p y6p phi6p...
     x1pp y1pp phi1pp x2pp y2pp phi2pp x3pp y3pp phi3pp...
     x4pp y4pp phi4pp x5pp y5pp phi5pp x6pp y6pp phi6pp...
     phi1pN phi2pN phi3pN phi4pN phi5pN phi6pN...
     lambda1 lambda2 lambda3 lambda4...
     lambda5 lambda6 lambda7 lambda8...
     lambda9 lambda10...
     u1 u2 u3 u4 u5...
     u1b u2b u3b u4b u5b...
     force1 force2 force3 force4 force5 force6 force7 force8 force9...
     force10 force11 force12 force13 force14 force15 force16 force17 force18

nb = 6;
n = nb*3;

% L1 = 1;
% L2 = 1;
% L3 = 1;
% m1 = 1;
% m2 = 1;
% m3 = 1;
% I1 = m1*((2*L1)^2)/3;
% I2 = m2*((2*L2)^2)/3;
% I3 = m3*((2*L3)^2)/3;

Lambda = [lambda1; lambda2; lambda3; lambda4; lambda5; lambda6; lambda7; lambda8; lambda9; lambda10];

Actuation = [-u1/2; u1/2-u2/2; u2/2-u3/2; u3/2-u4/2; u4/2-u5/2; u5/2];

%qp = [x1p;y1p;phi1p;x2p;y2p;phi2p;x3p;y3p;phi3p];
qpp1 = [x1pp;y1pp;phi1pp;x2pp;y2pp;phi2pp;x3pp;y3pp;phi3pp;...
        x4pp;y4pp;phi4pp;x5pp;y5pp;phi5pp;x6pp;y6pp;phi6pp];

nb = 6;         % Number of bodies
%n = nb*3;      % Number of generalized coordinates
m = (nb-1)*2;   % Number of constraints 

M1 = [m1, 0, 0;...
      0, m1, 0;...
      0, 0, I1];...
M2 = [m2, 0, 0;...
      0, m2, 0;...
      0, 0, I2];...
M3 = [m3, 0, 0;...
      0, m3, 0;...
      0, 0, I3];...
M4 = [m4, 0, 0;...
      0, m4, 0;...
      0, 0, I4];...
M5 = [m5, 0, 0;...
      0, m5, 0;...
      0, 0, I5];...
M6 = [m6, 0, 0;...
      0, m6, 0;...
      0, 0, I6];...
       
Z3 = zeros(3);
M = [M1,Z3,Z3,Z3,Z3,Z3;...
     Z3,M2,Z3,Z3,Z3,Z3;...
     Z3,Z3,M3,Z3,Z3,Z3;...
     Z3,Z3,Z3,M4,Z3,Z3;...
     Z3,Z3,Z3,Z3,M5,Z3;...
     Z3,Z3,Z3,Z3,Z3,M6];

% Jacobian (from Jac_Rev_Symbolic)
PHIq = [1, 0, cos(phi1), -1,  0, cos(phi2),  0,  0,         0,  0,  0,         0,  0,  0,         0,  0,  0,         0;...
        0, 1, sin(phi1),  0, -1, sin(phi2),  0,  0,         0,  0,  0,         0,  0,  0,         0,  0,  0,         0;...
        0, 0,         0,  1,  0, cos(phi2), -1,  0, cos(phi3),  0,  0,         0,  0,  0,         0,  0,  0,         0;...
        0, 0,         0,  0,  1, sin(phi2),  0, -1, sin(phi3),  0,  0,         0,  0,  0,         0,  0,  0,         0;...
        0, 0,         0,  0,  0,         0,  1,  0, cos(phi3), -1,  0, cos(phi4),  0,  0,         0,  0,  0,         0;...
        0, 0,         0,  0,  0,         0,  0,  1, sin(phi3),  0, -1, sin(phi4),  0,  0,         0,  0,  0,         0;...
        0, 0,         0,  0,  0,         0,  0,  0,         0,  1,  0, cos(phi4), -1,  0, cos(phi5),  0,  0,         0;...
        0, 0,         0,  0,  0,         0,  0,  0,         0,  0,  1, sin(phi4),  0, -1, sin(phi5),  0,  0,         0;...
        0, 0,         0,  0,  0,         0,  0,  0,         0,  0,  0,         0,  1,  0, cos(phi5), -1,  0, cos(phi6);...
        0, 0,         0,  0,  0,         0,  0,  0,         0,  0,  0,         0,  0,  1, sin(phi5),  0, -1, sin(phi6)];

PHIqT = transpose(PHIq);

% Force Vector
Q = [force1;force2;force3+Actuation(1);force4;force5;force6+Actuation(2);force7;force8;force9+Actuation(3);...
     force10;force11;force12+Actuation(4);force13;force14;force15+Actuation(5);force16;force17;force18+Actuation(6)];

% Gamma (from Gamm_rev_sym)
Gamma = [L1*sin(phi1)*phi1p^2 + L2*sin(phi2)*phi2p^2
        -L1*cos(phi1)*phi1p^2 - L2*cos(phi2)*phi2p^2
         L2*sin(phi2)*phi2p^2 + L3*sin(phi3)*phi3p^2
        -L2*cos(phi2)*phi2p^2 - L3*cos(phi3)*phi3p^2
         L3*sin(phi3)*phi3p^2 + L4*sin(phi4)*phi4p^2
        -L3*cos(phi3)*phi3p^2 - L4*cos(phi4)*phi4p^2
         L4*sin(phi4)*phi4p^2 + L5*sin(phi5)*phi5p^2
        -L4*cos(phi4)*phi4p^2 - L5*cos(phi5)*phi5p^2
         L5*sin(phi5)*phi5p^2 + L6*sin(phi6)*phi6p^2
        -L5*cos(phi5)*phi5p^2 - L6*cos(phi6)*phi6p^2];

% Zero Matrix
Zr22 = zeros(m,m);

LHS = [  M , PHIqT;...
       PHIq, Zr22];
RHS = [Q;Gamma];
Y1 = [qpp1;Lambda];

EQN1 = LHS*Y1 == RHS;

phi1ppN = -u1b/2;
phi2ppN = u1b/2-u2b/2;
phi3ppN = u2b/2-u3b/2;
phi4ppN = u3b/2-u4b/2;
phi5ppN = u4b/2-u5b/2;
phi6ppN = u5b/2;

qpp2 = [x1pp;y1pp;phi1ppN;x2pp;y2pp;phi2ppN;x3pp;y3pp;phi3ppN;...
        x4pp;y4pp;phi4ppN;x5pp;y5pp;phi5ppN;x6pp;y6pp;phi6ppN];
Y2 = [qpp2;Lambda];
EQN2 = LHS*Y2 == RHS;

% ------------------------------------

EQN2(3:3:n) = [];

ABvector = [x1pp,y1pp,x2pp,y2pp,x3pp,y3pp,x4pp,y4pp,x5pp,y5pp,x6pp,y6pp,lambda1,lambda2,lambda3,lambda4,...
            lambda5,lambda6,lambda7,lambda8,lambda9,lambda10];

[A,B] = equationsToMatrix(EQN2,ABvector);

X = linsolve(A,B);

Lambda = [X(13);X(14);X(15);X(16);X(17);X(18);X(19);X(20);X(21);X(22)];

Lambda

% ---------------------------------

eqn1 = EQN1(3);
eqn2 = EQN1(6);
eqn3 = EQN1(9);
eqn4 = EQN1(12);
eqn5 = EQN1(15);
eqn6 = EQN1(18);

[A,B] = equationsToMatrix([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6],[phi1pp,phi2pp,phi3pp,phi4pp,phi5pp,phi6pp]);
X = linsolve(A,B);

DPhi1 = collect(X(2)-X(1),[u1,u2,u3,u4,u5]);
DPhi2 = collect(X(3)-X(2),[u1,u2,u3,u4,u5]);
DPhi3 = collect(X(4)-X(3),[u1,u2,u3,u4,u5]);
DPhi4 = collect(X(5)-X(4),[u1,u2,u3,u4,u5]);
DPhi5 = collect(X(6)-X(5),[u1,u2,u3,u4,u5]);

[C,D] = equationsToMatrix([DPhi1,DPhi2,DPhi3,DPhi4,DPhi5],[u1,u2,u3,u4,u5]);
Y = linsolve(C,D);

u1 = collect(Y(1),[u1b,u2b,u3b,u4b,u5b]);
u2 = collect(Y(2),[u1b,u2b,u3b,u4b,u5b]);
u3 = collect(Y(3),[u1b,u2b,u3b,u4b,u5b]);
u4 = collect(Y(4),[u1b,u2b,u3b,u4b,u5b]);
u5 = collect(Y(5),[u1b,u2b,u3b,u4b,u5b]);

U = [u1;u2;u3;u4;u5];

% % Substitute Lambda
% lambdas = [lambda1,lambda2,lambda3,lambda4,lambda5,lambda6,lambda7,lambda8,lambda9,lambda10];
% Ui = subs(U,lambdas,[Lambda(1),Lambda(2),Lambda(3),Lambda(4),Lambda(5),Lambda(6),Lambda(7),Lambda(8),Lambda(9),Lambda(10)]);

% Don't substitute Lambda
Ui = U

end

